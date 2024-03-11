# Jamie Vendryes and Cooper Phillips
# EENG 350 - Section B
# Demo 1
'''
This code detects an aruco marker and then prints the angle
from the center of our camera to the aruco marker onto the
lcd screen.
'''

# Importing libraries
import threading
import queue
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
from smbus2 import SMBus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

fileName = "test.jpg"

# Initialize LCD stuff
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialize I2C bus
i2c = board.I2C()   # uses board.SCL and board.SDA

# Initialize the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)
	
# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
	
# Let the camera warmup
sleep(0.1)

# Reset LCD
lcd.clear()
lcd.message = ""
lastMessage = ""

# Initializing Variables
degree = 0
x_avg = 0
y_avg = 0
num = -1
temp = -50

# Initializing Queue
q = queue.Queue()

# Initializing Threading function
def myFunction():
    # Initialize LCD here
    lcd.clear()
    lcd.color = [100, 0, 0]

    while True:
        if not q.empty():
            degree = q.get()
            # Write new LCD data here
            lcd.message = "Angle: " + str(degree)

myThread = threading.Thread(target=myFunction, args=())
myThread.start()

# Get an image from the camera stream
while(1):
        # Detect the aruco marker
        ret, image = camera.read()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners,ids,rejected = aruco.detectMarkers(image,aruco_dict)
        # Calculate the center point of aruco marker
        if len(corners) > 0:
                x_avg = 0
                y_avg = 0
                for i in corners[0][0]:
                        x_avg += i[0]
                        y_avg += i[1]
                x_avg //= 4
                y_avg //= 4
                degree = ((x_avg - 320) / 320)*(62/2) #78 originally
                degree = round(degree, 2)

        # If degree has been set, use threading
        if temp != degree and (degree-1 > temp or degree+1 < temp):
                q.put(degree) 
                temp = degree

        overlay = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
        overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
        if not ids is None:
                ids = ids.flatten()
                for (outline, id) in zip(corners, ids):
                        markerCorners = outline.reshape((4,2))
                        overlay = cv2.putText(overlay, str(id), (int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2)
        k = cv2.waitKey(1)
        cv2.imshow("Overlay",overlay)
        if k == ord('q'):         # wait for ESC key to exit
                break

cv2.destroyAllWindows()
camera.release()
