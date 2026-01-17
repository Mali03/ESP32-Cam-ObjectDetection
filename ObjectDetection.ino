#include <Maden_suyu_detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"

#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// WiFi credentials
const char *ssid = "*******";
const char *password = "*******";

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

#define CAM_WIDTH EI_CAMERA_RAW_FRAME_BUFFER_COLS // 320
#define CAM_HEIGHT EI_CAMERA_RAW_FRAME_BUFFER_ROWS // 240

static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;

ei_impulse_result_bounding_box_t bb;

// To store detected objects
#define MAX_DETECTIONS 10
struct Detection {
  int x, y, w, h;
  char label[32];
  float confidence;
  bool valid;
};

Detection detections[MAX_DETECTIONS];
int detection_count = 0;
unsigned long last_detection_time = 0;

static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,

  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_RGB565, // for web stream use RGB565
  .frame_size = FRAMESIZE_QVGA, // 320x240

  .jpeg_quality = 12, //0-63 lower number means higher quality
  .fb_count = 1, // if more than one, i2s runs in continuous mode. Use only with JPEG
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_LATEST,
};

bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
void startCameraServer();
void drawRectangle(camera_fb_t *fb, int x, int y, int w, int h, uint16_t color);
void drawBoundingBoxes(camera_fb_t *fb);

/**
* @brief      Arduino setup function
*/
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  while (!Serial);

  for (int i = 0; i < MAX_DETECTIONS; i++) {
    detections[i].valid = false;
  }

  if (ei_camera_init() == false) {
    ei_printf("Failed to initialize Camera!\r\n");
  } else {
    ei_printf("Camera initialized\r\n");
  }

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  ei_printf("\nEveryting is ready object detection starting in 2 seconds...\n");
  ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop() {
  if (ei_sleep(5) != EI_IMPULSE_OK) return;

  snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

  // check if allocation was successful
  if (snapshot_buf == nullptr) {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
    ei_printf("Failed to capture image\r\n");
    free(snapshot_buf);
    return;
  }

  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  // print the predictions
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n", result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  ei_printf("Object detection bounding boxes:\r\n");

  detection_count = 0;

  for (uint32_t i = 0; i < result.bounding_boxes_count && i < MAX_DETECTIONS; i++) {
    bb = result.bounding_boxes[i];

    // Confidence threshold
    if (bb.value < 0.5) {
      ei_printf("  Skipping low confidence detection: %s (%.2f)\r\n", bb.label, bb.value);
      continue;
    }

    float scale_x = (float)CAM_WIDTH / (float)EI_CLASSIFIER_INPUT_WIDTH;
    float scale_y = (float)CAM_HEIGHT / (float)EI_CLASSIFIER_INPUT_HEIGHT;

    ei_printf("Scale factors: x=%.3f, y=%.3f\r\n", scale_x, scale_y);

    const float COMPENSATION_FACTOR = 0.7;
    const float BOX_SIZE_MULTIPLIER = 3.0;

    int base_w = (int)round(bb.width * scale_x * COMPENSATION_FACTOR);
    int base_h = (int)round(bb.height * scale_y * COMPENSATION_FACTOR);

    int real_w = (int)round(base_w * BOX_SIZE_MULTIPLIER);
    int real_h = (int)round(base_h * BOX_SIZE_MULTIPLIER);

    int base_x = (int)round(bb.x * scale_x);
    int base_y = (int)round(bb.y * scale_y);
    int real_x = base_x - (real_w - base_w) / 2;
    int real_y = base_y - (real_h - base_h) / 2;

    const int MIN_BOX_SIZE = 10;
    if (real_w < MIN_BOX_SIZE || real_h < MIN_BOX_SIZE) {
      ei_printf("  Skipping too small detection: %s (%dx%d)\r\n", bb.label, real_w, real_h);
      continue;
    }

    if (real_x < 0) {
      real_w += real_x;
      real_x = 0;
    }
    if (real_y < 0) {
      real_h += real_y;
      real_y = 0;
    }
    if (real_x + real_w > CAM_WIDTH) real_w = CAM_WIDTH - real_x;
    if (real_y + real_h > CAM_HEIGHT) real_h = CAM_HEIGHT - real_y;

    if (real_w <= 0 || real_h <= 0) {
      ei_printf("  Invalid box after bounds check: %s\r\n", bb.label);
      continue;
    }

    // Store detection informations
    detections[detection_count].x = real_x;
    detections[detection_count].y = real_y;
    detections[detection_count].w = real_w;
    detections[detection_count].h = real_h;
    detections[detection_count].confidence = bb.value;
    strncpy(detections[detection_count].label, bb.label, 31);
    detections[detection_count].label[31] = '\0';
    detections[detection_count].valid = true;
    detection_count++;

    ei_printf(
      "  %s (%f) [ Original: x: %.1f, y: %.1f, w: %.1f, h: %.1f ] [ Scaled: x: %d, y: %d, w: %d, h: %d ]\r\n",
      bb.label,
      bb.value,
      bb.x,
      bb.y,
      bb.width,
      bb.height,
      real_x,
      real_y,
      real_w,
      real_h);
  }

  last_detection_time = millis();

#else
  ei_printf("Predictions:\r\n");
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
    ei_printf("%.5f\r\n", result.classification[i].value);
  }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
  ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
  ei_printf("Visual anomalies:\r\n");
  for (uint32_t i = 0; i < result.visual_ad_count; i++) {
    bb = result.visual_ad_grid_cells[i];
    if (bb.value == 0) {
      continue;
    }

    float scale_x = (float)CAM_WIDTH / (float)EI_CLASSIFIER_INPUT_WIDTH;
    float scale_y = (float)CAM_HEIGHT / (float)EI_CLASSIFIER_INPUT_HEIGHT;

    int real_x = (int)(bb.x * scale_x);
    int real_y = (int)(bb.y * scale_y);
    int real_w = (int)(bb.width * scale_x);
    int real_h = (int)(bb.height * scale_y);

    if (real_x < 0) real_x = 0;
    if (real_y < 0) real_y = 0;
    if (real_x + real_w > CAM_WIDTH) real_w = CAM_WIDTH - real_x;
    if (real_y + real_h > CAM_HEIGHT) real_h = CAM_HEIGHT - real_y;

    ei_printf(
      "  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
      bb.label,
      bb.value,
      real_x,
      real_y,
      real_w,
      real_h);
  }
#endif
  free(snapshot_buf);
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

  if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, 0);
  }

  is_initialised = true;
  return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {
  esp_err_t err = esp_camera_deinit();

  if (err != ESP_OK) {
    ei_printf("Camera deinit failed\n");
    return;
  }

  is_initialised = false;
  return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  bool do_resize = false;

  if (!is_initialised) {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    ei_printf("Camera capture failed\n");
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_RGB565, snapshot_buf);

  esp_camera_fb_return(fb);

  if (!converted) {
    ei_printf("Conversion failed\n");
    return false;
  }

  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
      || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
    do_resize = true;
  }

  if (do_resize) {
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height);
  }

  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    // Swap BGR to RGB here
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

    // Go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }

  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif

/**
 * @brief Set a pixel in RGB565
 */
void setPixelRGB565(uint8_t *fb, int width, int x, int y, uint16_t color) {
  if (x < 0 || y < 0 || x >= width) return;
  int index = (y * width + x) * 2;
  fb[index] = color & 0xFF;
  fb[index + 1] = (color >> 8) & 0xFF;
}

/**
 * @brief Draw a rectangle (in RGB565 format)
 */
void drawRectangle(uint8_t *fb, int fb_width, int fb_height, int x, int y, int w, int h, uint16_t color, int thickness) {
  if (x < 0) {
    w += x;
    x = 0;
  }
  if (y < 0) {
    h += y;
    y = 0;
  }
  if (x + w > fb_width) w = fb_width - x;
  if (y + h > fb_height) h = fb_height - y;

  if (w <= 0 || h <= 0) return;

  for (int t = 0; t < thickness; t++) {
    for (int i = x; i < x + w; i++) {
      if (y + t < fb_height)
        setPixelRGB565(fb, fb_width, i, y + t, color);
      if (y + h - 1 - t >= 0 && y + h - 1 - t < fb_height)
        setPixelRGB565(fb, fb_width, i, y + h - 1 - t, color);
    }
  }

  for (int t = 0; t < thickness; t++) {
    for (int j = y; j < y + h; j++) {
      if (x + t < fb_width)
        setPixelRGB565(fb, fb_width, x + t, j, color);
      if (x + w - 1 - t >= 0 && x + w - 1 - t < fb_width)
        setPixelRGB565(fb, fb_width, x + w - 1 - t, j, color);
    }
  }
}

/**
 * @brief Draw rectangles around all detected objects and write labels.
 */
void drawBoundingBoxes(camera_fb_t *fb) {
  if (!fb || fb->format != PIXFORMAT_RGB565) {
    Serial.println("FB is null or wrong format!");
    return;
  }

  Serial.printf("Drawing %d detections on %dx%d frame\n", detection_count, fb->width, fb->height);

  fb_data_t fb_data;
  fb_data.width = fb->width;
  fb_data.height = fb->height;
  fb_data.data = fb->buf;
  fb_data.bytes_per_pixel = 2;
  fb_data.format = FB_RGB565;

  for (int i = 0; i < detection_count; i++) {
    if (detections[i].valid) {
      Serial.printf("  Drawing box %d: x=%d, y=%d, w=%d, h=%d, conf=%.2f\n", i, detections[i].x, detections[i].y, detections[i].w, detections[i].h, detections[i].confidence);

      drawRectangle(
        fb->buf,
        fb->width,
        fb->height,
        detections[i].x,
        detections[i].y,
        detections[i].w,
        detections[i].h,
        0x07E0,
        2);

      char label_text[64];
      snprintf(label_text, sizeof(label_text), "%s %.0f%%",
               detections[i].label,
               detections[i].confidence * 100);

      int text_x = detections[i].x + 2;
      int text_y = detections[i].y - 2;

      if (text_y < 10) {
        text_y = detections[i].y + 10;
      }

      fb_gfx_print(&fb_data, text_x, text_y, 0x07E0, label_text);
    }
  }
}

// Constants for web server
static httpd_handle_t camera_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[64];

  static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
  static const char *_STREAM_BOUNDARY = "\r\n--frame\r\n";
  static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->format == PIXFORMAT_RGB565) {
        drawBoundingBoxes(fb);
      }

      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          Serial.println("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }

    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, part_buf, hlen);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      }
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if (res != ESP_OK) {
      break;
    }
  }

  return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
  const char *html =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>ESP32-CAM Object Detection</title>"
    "<style>body{margin:0;padding:20px;background:#1a1a1a;color:#fff;font-family:Arial,sans-serif;text-align:center}"
    "h1{color:#4CAF50}img{max-width:100%;height:auto;border:3px solid #4CAF50;border-radius:8px}</style>"
    "</head><body>"
    "<h1>ESP32-CAM Object Detection</h1>"
    "<img src='/stream' id='stream'>"
    "</body></html>";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &stream_uri);
    Serial.println("Camera server started");
  } else {
    Serial.println("Failed to start camera server");
  }
}
