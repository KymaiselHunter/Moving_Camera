import cv2 
from matplotlib import pyplot as plt
import numpy as np

from PIL import Image

#print("test")

def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit
cap = cv2.VideoCapture(0)

while cap.isOpened():
  ret, frame = cap.read()

  if not ret:
    break
    #print(frame)

  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

  #========
  #color mask
  #========
  hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  
  low,high = get_limits([0,255,255])

  mask = cv2.inRange(hsvImage,low,high)

  mask_ = Image.fromarray(mask)

  bbox = mask_.getbbox()

  centerScreenX = int(len(frame[0])/2)
  centerScreenY = int(len(frame)/2)

  if bbox is not None:
      x1, y1, x2, y2 = bbox

      frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

      centerRecX = int((x1+x2)/2)
      centerRecY = int((y1+y2)/2)

      #print(centerScreenX, centerScreenY)

      cv2.line(frame, (centerScreenX, centerScreenY), (centerRecX, centerRecY), (255, 0, 0), 5)

      location = ""
      
      if centerScreenY > centerRecY:
         location += "upper "
      else:
         location += "lower "

      if centerScreenX > centerRecX:
         location += "left"
      else:
         location += "right"

      print(location)



  

  #for i in range(len(frame)):
  #  for j in range(len(frame[i])):
      

  #    if frame[i][j][2] < 240:
  #      frame[i][j] = 0
  #      frame[i][j][0]=0
  #      frame[i][j][1]=0



  #========
  # display
  #========
  cv2.imshow('masked', mask)
  cv2.imshow('cam',frame)#cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
  #break
  #plt.show()


cap.release()
cv2.destroyAllWindows()