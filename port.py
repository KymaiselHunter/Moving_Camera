#import serial.tools.list_ports

#ports = serial.tools.list_ports.comports()
#serialInst = serial.Serial()

#portList = []

#for onePort in ports:
#  portList.append(str(onePort))
#  print(str(onePort))

#serialInst.baudrate = 9600
#serialInst.port = ports[0].device
#serialInst.open()

#while True:
#  if serialInst.in_waiting:
#    packet = serialInst.readline()
#    print(packet.decode('utf'))

import serial.tools.list_ports

import keyboard # for testing

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

#portList = []

#for onePort in ports:
#  portList.append(str(onePort))

serialInst.baudrate = 9600
serialInst.port = ports[0].device
serialInst.open()

holdW = False
holdS = False

while True:
  #serialInst.write(input("guh: ").encode('utf-8'))
  ##print("test")
  
  ##out = "180"
  ##serialInst.write(out.encode('utf-8'))
  #continue
  if keyboard.is_pressed('w') and not holdW:
    print("hello???")
    out = "180"
    serialInst.write(out.encode('utf-8'))
    holdW = True
    continue
  elif not keyboard.is_pressed('w'):
    holdW = False

  if keyboard.is_pressed('s') and not holdS:
    print("hello2???")
    out = "0"
    serialInst.write(out.encode('utf-8'))
    holdS = True
    continue
  elif not keyboard.is_pressed('s'):
    holdS = False


  if keyboard.is_pressed('q'):
    break

serialInst.close()