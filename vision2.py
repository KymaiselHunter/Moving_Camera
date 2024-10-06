# import serial for access to arduino
import serial.tools.list_ports

#Import the following classes to access the Face Detector task functions:
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import utilities as util

#basic cv2 stuff
import cv2 

#port set up
ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

serialInst.baudrate = 9600
serialInst.port = ports[0].device
serialInst.open()

#ready media pipe
BaseOptions = mp.tasks.BaseOptions
FaceDetector = mp.tasks.vision.FaceDetector
FaceDetectorOptions = mp.tasks.vision.FaceDetectorOptions
FaceDetectorResult = mp.tasks.vision.FaceDetectorResult
VisionRunningMode = mp.tasks.vision.RunningMode

# recent Data class is just a nice way of storing the most recent face detection
# data since it runs asyncynously
class RecentData:
  def __init__(self):
    self.results = None
    self.time = None

  def top():
    return holdRes, holdTime
  
  def pop(self):
    if self.results:
      holdRes = self.results
      holdTime = self.time

      self.results = None
      self.time = None

      return holdRes, holdTime
    else:
      return False, False
  
  def push(self,result, time):
    self.results = result
    self.time = time


# instantiate the data holder
dataHold = RecentData()

up = 0

# Create a face detector instance with the live stream mode:
# on the documentation, this was initally print results
# i changed to process since i want this data back in the while loop
# attempting to draw output inside this function resulted in terrible framerate
# so having it in the while loop, grabbing what ever was in data hold
# produced a much nicer outcome
def process_result(result: FaceDetectorResult, output_image: mp.Image, timestamp_ms: int):
    #print('face detector result: {}'.format(result))
    if result.detections:
      dataHold.push(result, timestamp_ms)
    #print(" ")
    

#set for the face detection on google's documentations
options = FaceDetectorOptions(
    base_options=BaseOptions(model_asset_path='Models/blaze_face_short_range.tflite'),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=process_result)
with FaceDetector.create_from_options(options) as detector:
  # The detector is initialized. Use it here.
  # ...
  
  #grab the cam to be used
  cap = cv2.VideoCapture(1)

  # Use OpenCV’s VideoCapture to start capturing from the webcam.
  while cap.isOpened():

  # Create a loop to read the latest frame from the camera using VideoCapture#read()
    ret, frame = cap.read()

    if not ret:
      continue

    #quit program via "q" press
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break


    #grab center of screen for calculations
    centerScreenX = int(len(frame[0])/2)
    centerScreenY = int(len(frame)/2)


    # Convert the frame received from OpenCV to a MediaPipe’s Image object.
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

    # Send live image data to perform face detection.
    # The results are accessible via the `result_callback` provided in
    # the `FaceDetectorOptions` object.
    # The face detector must be created with the live stream mode.
    frame_timestamp_ms = int(cap.get(cv2.CAP_PROP_POS_MSEC))

    detector.detect_async(mp_image, frame_timestamp_ms)

    #========
    # display
    #========
    res, time = dataHold.pop()

    if res and 100 > abs(time-frame_timestamp_ms):
      frame = util.visualize(frame,res)
      print()
      print(res)

      centerScreenX = int(len(frame[0])/2)
      centerScreenY = int(len(frame)/2)

      detection = res.detections[0]

      centerRecX = int((detection.bounding_box.origin_x+detection.bounding_box.height/2))
      centerRecY = int((detection.bounding_box.origin_y+detection.bounding_box.width/2))

      #print(centerScreenX, centerScreenY)

      cv2.line(frame, (centerScreenX, centerScreenY), (centerRecX, centerRecY), (255, 255, 0), 3)

      if centerRecY > centerScreenY:
        if up != -1:
          serialInst.write(str(180).encode('utf-8'))
          up = -1
      elif centerRecY < centerScreenY:
        if up != 1:
          serialInst.write(str(0).encode('utf-8'))
          up = 1
      #serialInst.write(str((centerRecY / len(frame)) * 180).encode('utf-8'))

      cv2.imshow('cam', frame)
    else:
      cv2.imshow('cam',frame)#cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))


  # release camera
  cap.release()
  cv2.destroyAllWindows()

  #release port
  serialInst.close()