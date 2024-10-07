# Moving Camera with Face Detection!

## Overview
This repository contains the code for my moving camera project, combining arduino parts and a python script handled on my laptop for it to function:  
[Youtube Shorts Preview](https://youtube.com/shorts/-iFjHyCmI6o)  

## Libraries and APIs
- For the camera, **openCV** handles the basics of Computer Vision, grabbing data from a live webcam and allowing these frames to be analyzed and annotated  
- Google's incredibly powerful [MediaPipe](https://ai.google.dev/edge/mediapipe/solutions/vision/face_detector/python) is used to detect faces within the given frame  
- To communicate between the python script and the arduino, Serial Tools are used to transmit data through a wired connection at any of the computer's Serial Ports  

### Preparation and Installation
To use this project, one must prepare two main components of the projects.
  - Python
  - Arduino

### Python Code and Dependencies Dependencies:
Assuming you have python already installed, you just need to clone the repo and download the dependencies

```bash
  git clone git@github.com:KymaiselHunter/Moving_Camera.git
  cd Moving_Camera

  pip install opencv-python mediapipe pyserial
   ```

### Arduino/Real life:
  - Make any base for a webcam/camera to sit on with a y and x axis a mortor can spin: [I used cardboard, but this simple version is what inspired me](https://youtube.com/shorts/B-VihJLjGr4?si=PQqd_AEZc5qyewnC)  
  - Set up an arduino connected to two servo motors: [Servo Motor Tutorial](https://youtu.be/tRNcEzlCOdo?si=kQcFkOxiPwGD7jdV)
  - Connect a video camera to a computer and sattle it on the base
  - Upload the Arduino Code (that you may have already retrieved by cloning the repo) to Arduino through any serial port so it can [interpret the python script instructions and control the motors](https://github.com/KymaiselHunter/Moving_Camera/blob/main/vision2.py)


