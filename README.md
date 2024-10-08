# Moving Camera with Face Detection!

## Overview
This repository contains the code for my moving camera project, combining arduino parts and a python script handled on my laptop to create a moving camera that locks onto faces within frame.  

[Youtube Shorts Preview](https://youtube.com/shorts/-iFjHyCmI6o)  

## Libraries and APIs
- For the camera, **openCV** handles the basics of Computer Vision, grabbing data from a live webcam and allowing these frames to be analyzed and annotated  
- Google's incredibly powerful [MediaPipe (with an amazing guide for it linked)](https://ai.google.dev/edge/mediapipe/solutions/vision/face_detector/python) is used to detect faces within the given frame  
- To communicate between the python script and the arduino, Serial Tools are used to transmit data through a wired connection at any of the computer's Serial Ports  

## Arduino Hardware
  -  [ELEGOO UNO R3 Board ATmega328P with USB Cable(Arduino-Compatible) for Arduino](https://www.amazon.com/gp/product/B01EWOE0UU/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
  -  [Two Servo Motors](https://www.amazon.com/gp/product/B0BKPL2Y21/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1) (for X and Y axis control)
  -  [Breadboard](https://www.amazon.com/gp/product/B07LFD4LT6/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1) and Wires
  -  Camera/Webcam (to capture video input, any camera that can work as a webcam should work)
  -  Computer (for running the Python script, powering Arduino and grabbing camera data)

### Preparation and Installation
To use this project, one must prepare two main components of the projects.
  1. Python
  2. Arduino

### Python Code and Dependencies Dependencies:
Assuming you have python already installed, you just need to clone the repo and download the dependencies

```bash
  git clone git@github.com:KymaiselHunter/Moving_Camera.git
  cd Moving_Camera

  pip install opencv-python mediapipe pyserial
   ```

### Arduino/Real life:
  - Make any base for a webcam/camera to sit on with a y and x axis a mortor can spin: [I used cardboard, but this simple version is what inspired me](https://youtube.com/shorts/B-VihJLjGr4?si=PQqd_AEZc5qyewnC)  
  - Set up an arduino connected to two servo motors with the y-axis motor connected to pin 3 and x-axis connected to pin 5: [Servo Motor Tutorial](https://www.youtube.com/watch?v=QbgTl6VSA9Y)
  - Connect a video camera to a computer and sattle it on the base
  - Upload the Arduino Code (that you may have already retrieved by cloning the repo) to Arduino through any serial port so it can [interpret the python script instructions and control the motors](https://github.com/KymaiselHunter/Moving_Camera/blob/main/computer_to_servo/computer_to_servo.ino)


## Usage
Once set up, the code you may run the code 
   ```bash
   python main.py
   ```
The python script and ardunio will handle everythin in realtime, hence running the code is sufficient as all the hard work was in the preparation. Thus, if you got this far, good job, hopefully it works and enjoy! >:p
