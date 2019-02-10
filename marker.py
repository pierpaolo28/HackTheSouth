import numpy as np
import cv2
import cv2.aruco as aruco
import time
import serial
from PIL import Image

MAX_AREA = 640*480

cap = cv2.VideoCapture(0)

#ser = serial.Serial('/dev/ttyACM0', baudrate=9600,
#                    parity=serial.PARITY_NONE,
#                    stopbits=serial.STOPBITS_ONE,
#                    bytesize=serial.EIGHTBITS
#                    )       
port = serial.Serial()
port.port = '/dev/ttyUSB1'
port.baud = 9600        
port.open()  
time.sleep(2)

def PolyArea(x,y):
    retval = 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))
    retval = 255 * retval / MAX_AREA
    return retval

def avg_x(corners):
    retval = np.mean(corners[:, 0])
    retval = 255 * retval / 640
    return retval   
   
def area_sq(corners):
    return PolyArea(corners[:, 0], corners[:, 1])

def gen_string(ids, corners):
    final = []
    if ids is None:
        final = [91, 0, 44, 0, 44, 0, 93]
    else:
        for i in range(len(ids)):
            a = int(ids[i]) & 0xff
            b = int(avg_x(corners[i][0])) & 0xff
            c = int(area_sq(corners[i][0])) & 0xff
            # print(a, b, c)
            final.extend([91, a, 44, b, 44, c, 93])
            print(2.0 * b / 255.0 - 1.0)
    return bytes(final)                    

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()

    ''' detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
    '''
    
    # [tag_id, x_pos, area]
    
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(ids)
    #port = serial.Serial()
    #port.port = '/dev/ttyACM0'
    #port.baud = 9600
    #with port as ser:
    #    ser.write(gen_string(ids, corners))
    port.write(gen_string(ids, corners))
    time.sleep(0.001)

    gray = aruco.drawDetectedMarkers(gray, corners)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
port.close()
cap.release()
cv2.destroyAllWindows()
