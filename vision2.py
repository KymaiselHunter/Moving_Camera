#Import the following classes to access the Face Detector task functions:
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import utilities as util

#basic cv2 stuff
import cv2 

#ready media pipe
BaseOptions = mp.tasks.BaseOptions
FaceDetector = mp.tasks.vision.FaceDetector
FaceDetectorOptions = mp.tasks.vision.FaceDetectorOptions
FaceDetectorResult = mp.tasks.vision.FaceDetectorResult
VisionRunningMode = mp.tasks.vision.RunningMode

class RecentData:
  def __init__(self):
    self.results = None
    self.time = None

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

dataHold = RecentData()

# Create a face detector instance with the live stream mode:
def print_result(result: FaceDetectorResult, output_image: mp.Image, timestamp_ms: int):
    #print('face detector result: {}'.format(result))
    if result.detections:
      dataHold.push(result, timestamp_ms)
    #print(" ")
    


options = FaceDetectorOptions(
    base_options=BaseOptions(model_asset_path='Models/blaze_face_short_range.tflite'),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=print_result)
with FaceDetector.create_from_options(options) as detector:
  # The detector is initialized. Use it here.
  # ...
    
  cap = cv2.VideoCapture(1)

  # Use OpenCV’s VideoCapture to start capturing from the webcam.
  while cap.isOpened():

  # Create a loop to read the latest frame from the camera using VideoCapture#read()
    ret, frame = cap.read()

    if not ret:
      continue

    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

    bbox = None

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
      cv2.imshow('cam',util.visualize(frame,res))
    else:
      cv2.imshow('cam',frame)#cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))


  cap.release()
  cv2.destroyAllWindows()