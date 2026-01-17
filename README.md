# ESP32-Cam Edge Impulse Object Detection

This project uses an ESP32-CAM (AI Thinker) module to run an Edge Impulse–trained AI object detection model and stream the results live through a web browser.

Camera frames are processed directly on the ESP32, objects are detected using the AI model, and bounding boxes with labels and confidence scores are drawn on the live video stream.

<p float="left">
  <img src="/Result Example 1.png" width="350" />
  <img src="/Result Example 2.png" width="350" />
</p>

## Installation

### Getting Ready
To program the ESP32-CAM, follow these steps:
1) Open Arduino IDE -> File -> Preferences
2) Add following URL to **Additional boards manager URLs**;
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

### Collecting Data to Train the AI Model
1) Complete the [Getting Ready](#getting-ready) steps
2) Open Arduino IDE -> Library Manager
3) Install the `EloquentEsp32Cam` library
4) Go File -> Examples -> EloquentEsp32Cam -> `Collect_Images_for_EdgeImpulse`
5) Upload the code to tje ESP32-CAM (For programming, see [Programming ESP32 Cam](#programming-esp32-cam))
6) Collect images of the object to be detected (collect at least 50 images)

### Programming ESP32 Cam
The ESP32-CAM does not include an integrated USB-to-serial programmer so an external FTDI (USB-to-TTL) programmer is used for uploading code. (You can also use an Arduino Uno as a USB-to-serial adapter. For more information, refer to tutorial videos on YouTube.)
<p float="left">
  <img src="/FTDI Programmer Pinout.png" width="500" />
</p>

### Train an AI model with Edge Impulse
1) Go to **https://studio.edgeimpulse.com/**
2) Create new project
3) Navigate to **Data acquisition -> Add data -> Upload data**
4) Select the folder containing the collected images
5) Go to label section and choose **Enter label**
6) Enter your object name
7) Click on upload data button
8) Go **Labelling Queue** and label your images
9) Navigate to **Create Impulse**
10) Add an **Image Processing** block
11) Add an **Object Detection** learning block
12) Save the impulse
13) Go to **Image** and set **Color depth** to **Grayscale**
14) Save parameters and click **Generate Features**
15) Go to **Object Detection** and set the **Learning rate** to `0.01`
16) Click **Start Training**
17) After training is complete, go to **Deployment**
18) Select **Arduino Library** as the deployment option
19) Set the target to **Espressif ESP-EYE (ESP32 240MHz)**
20) Click **Build** and download the generated library
21) In Arduino IDE, go to Sketch → Include Library → Add .ZIP Library
22) Open `ObjectDetection.ino` and replace
 ```
 #include <Maden_suyu_detection_inferencing.h>
 ```
 with your own model header file
23) Upload the code and open the Serial Monitor at **115200 baud**
24) Enter the ESP32-CAM IP address in your browser — and you're done!
