# Cooper Phillips and Jamie Vendryes 
# EENG 350 - Section B
# Mini Project Assignment

'''
This is the Python code that will utilize the camera and LCD screen by using a Pi computer.
The camera will be used to display video feed and to find the proper coordinates. Once found,
the coordinated will be displayed on the LCD screen without any lag by using threading,
then these coordinates will be sent to the Arduino for further use (wheel rotation).
'''

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

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8

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
x_avg = 0
y_avg = 0
num = -1
temp = -1
offset = 0
locations = ['[0, 0]', '[0, 1]', '[1, 0]', '[1, 1]']

# Initializing Queue
q = queue.Queue()

# Initializing Threading function
def myFunction():
    # Initialize LCD here
    lcd.clear()
    lcd.color = [100, 0, 0]

    while True:
        if not q.empty():
            num = q.get()
            # Write new LCD data here
            lcd.message = "Desired Location:\n" + locations[num]

myThread = threading.Thread(target=myFunction, args=())
myThread.start()

# Get an image from the camera stream
while(1):
        ret, image = camera.read()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners,ids,rejected = aruco.detectMarkers(image,aruco_dict)
        if len(corners) > 0:
                for i in corners[0][0]:
                        x_avg += i[0]
                        y_avg += i[1]
                x_avg //= 4
                y_avg //= 4
                # print(x_avg, ",", y_avg)
                if x_avg < 320 and y_avg < 240:         # NW, (0, 1)
                        num = 1
                elif x_avg < 320:                       # SW, (1, 1)
                        num = 3
                elif x_avg > 320 and y_avg > 240:       # SE, (1, 0)
                        num = 2
                else:                                   # NE, (0, 0)
                        num = 0
      
        # If num has been set, use threading
        if num >= 0 and temp != num:
                q.put(num) 
                temp = num 

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

        # arduino send code
        i2c.write_byte_data(ARD_ADDR, offset, num)
        sleep(0.1)

cv2.destroyAllWindows()
camera.release()
