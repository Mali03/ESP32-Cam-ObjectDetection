# ESP32-CAM Edge Impulse Object Detection

![ESP32](https://img.shields.io/badge/ESP32-blue?logo=espressif)
![Edge Impulse](https://img.shields.io/badge/Edge%20Impulse-blueviolet?logo=edgeimpulse)
![Computer Vision](https://img.shields.io/badge/Computer%20Vision-Object%20Detection-blue)
![Deep Learning](https://img.shields.io/badge/Deep%20Learning-TinyML-orange)
![IoT](https://img.shields.io/badge/IoT-green?logo=iot)

This project uses an ESP32-CAM (AI Thinker) module to run an Edge Impulse–trained AI object detection model and stream the results live through a web browser.

Camera frames are processed directly on the ESP32, objects are detected using the AI model, and bounding boxes with labels and confidence scores are drawn on the live video stream.

<p float="left">
  <img src="/Result Example 1.png" width="350" />
  <img src="/Result Example 2.png" width="350" />
</p>

## Contents
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Known Limitations](#known-limitations)
- [Installation](#installation)
- [License](#license)
- [Need help](#need-help)

## Features
- ESP32-CAM (AI Thinker) support
- Object Detection with Edge Impulse
- Bounding boxes drawn on live video
- Built-in web server
- Real-time MJPEG streaming (/stream endpoint)

## Hardware Requirements
- ESP32-CAM (AI Thinker)
- USB-to-TTL adapter (FTDI, CP2102, etc.)
- Jumper wires

## Known Limitations
- Higher resolutions reduce FPS
- Many detections increase RAM usage
- ESP32-CAM has limited compute resources for large models

## Installation

### Getting Ready
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
5) Upload the code to tje ESP32-CAM (For programming, see [Programming ESP32-CAM](#programming-esp32-cam))
6) Collect images of the object to be detected (collect at least 50 images)

### Programming ESP32-CAM
The ESP32-CAM does not include an integrated USB-to-serial programmer so an external FTDI (USB-to-TTL) programmer is used for uploading code. (You can also use an Arduino Uno as a USB-to-serial adapter. For more information, refer to tutorial videos on YouTube.)
<p float="left">
  <img src="FTDI Programmer Pinout.png" width="500" />
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
```cpp
#include <Maden_suyu_detection_inferencing.h>
```
with your own model header file

Update the WiFi credentials in the code before uploading it to the ESP32-CAM:
```cpp
const char *ssid = "*******";
const char *password = "*******";
```
23) Upload the code and open the Serial Monitor at **115200 baud** and copy the ip address
25) Open a browser and navigate to:
```
http://ESP32_IP_ADDRESS/
```
You will see the live camera stream with detected objects highlighted.

## License
This project is licensed under the **MIT License** - see the [LICENSE](https://github.com/Mali03/ESP32-CAM-ObjectDetection/blob/main/LICENSE) file for details.

## Need Help
If you need any help contact me on [LinkedIn](https://www.linkedin.com/in/mali03/).

⭐ If you like this project, don’t forget to give it a star on GitHub!
