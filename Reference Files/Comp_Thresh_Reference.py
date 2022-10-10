import logging
from typing import Any
import cv2
import threading

from networktables import NetworkTables as nt
from cscore import CameraServer

"""
COMPETITION VERSION
comp_thresh.py: Thresholds an image and finds contours and detects balls for FRC 2022 live from NetworkTables with RasberryPi 4 
    inspired from: https://github.com/rebels2638/frc-vision-2022/blob/main/countour_threshold_method/threshold_contours.py, https://stackoverflow.com/questions/58016325/thinness-ratio-calculation-wrong-results?noredirect=1&lq=1
"""

__author__ = ["Caden Li","Kushagra Saxena","Anant Gupta"]
__copyright__ = ["Copyright 2022, Kushagra Saxena & Caden Li"]
__credits__ = ["Collin Li"]
__license__ = "MIT"

########################################################################################

# --------NETWORKTABLES CONNECTION START--------

logging.basicConfig(level=logging.DEBUG)

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):

    print(info, '; Connected=%s' % connected)

    with cond:
        notified[0] = True
        cond.notify()

nt.initialize(server="10.26.38.2")
nt.addConnectionListener(connectionListener, immediateNotify=True)

with cond:

    print("Waiting")

    if not notified[0]:
        cond.wait()

print("Connected")

# --------NETWORKTABLES CONNECTION END--------

########################################################################################

# --------Camera Inputs START--------

cs = CameraServer.getInstance()

table = nt.getTable('Tracked Balls') #SmartDashboard
table2 = nt.getTable('SmartDashboard/DB')

height = 180 # 300
width = 320 # 400

outputStreamRedmask = cs.putVideo("Red mask",width,height)
outputStreamBlueMask = cs.putVideo("Blue mask",width,height)
outputStream = cs.putVideo("Frame",width,height)

camera = cv2.VideoCapture(-1) # try 1?

camera.set(3,width)
camera.set(4,height)
camera.set(10,0.40)

# --------Camera Inputs END--------

########################################################################################

# --------COLORS START--------

colors = {'blue':(255,0,0),'red':(0,0,255)}

lower = {'blue':(90,126,30),'red':(0,89,40)}  # get these two right already whats taking so long, bruhmmt
upper = {'blue':(130,255,255),'red':(10,255,255)} #red upper bounds: 170 102 102 & 179 255 255

# Red is in 2 ranges (0-10, 170-180) 
lower2 = {'red':(170,102,90)}
upper2 = {'red':(180,255,255)}

#Inputs default values to NetworkTables
table = nt.getTable('Tracked Balls') # SmartDashboard

table2.putValue("String 2",str([lower['red'][0],lower['red'][1],lower['red'][2]])[1:-1])
table2.putValue("String 3",str([lower2['red'][0],lower2['red'][1],lower2['red'][2]])[1:-1])
table2.putValue("String 4",str([upper['red'][0],upper['red'][1],upper['red'][2]])[1:-1])
table2.putValue("String 5",str([upper2['red'][0],upper2['red'][1],upper2['red'][2]])[1:-1])

table2.putValue("String 0",str([lower['blue'][0],lower['blue'][1],lower['blue'][2]])[1:-1])
table2.putValue("String 1",str([upper['blue'][0],upper['blue'][1],upper['blue'][2]])[1:-1])

# --------COLORS END--------

########################################################################################

# --------LIVE CAMERA INPUT START--------

while True:

    frame = camera.read()[1]  

    #Takes in frame and converts it to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)    

    # Loops through each color (Red, Blue) and creates masks for each frame based on color
    for key in colors.keys():
    
        mask = cv2.inRange(hsv, lower[key], upper[key]) 

        #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (5,5), iterations = 2)
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, (3,3), iterations = 4)

        if key == 'red': #If red, then we must add in the 2nd range that Red contains in the HSV colorspace

            mask += cv2.inRange(hsv, lower2[key], upper2[key])
            outputStreamRedmask.putFrame(mask)
        else:
            outputStreamBlueMask.putFrame(mask)
    
    # Displays default frame for comparison (light, shadows, etc.)
    outputStream.putFrame(frame)

    # # Gets new input values for HSV Range from NetworkTables 

    blue_range_lower = list(map(int, table2.getValue('String 0', Any).split(','))) #blue_range_lower
    blue_range_upper = list(map(int, table2.getValue('String 1', Any).split(','))) #blue_range_upper

    red_range_lower1 = list(map(int, table2.getValue('String 2', Any).split(','))) #red_range_lower1
    red_range_lower2 = list(map(int, table2.getValue('String 3', Any).split(','))) #red_range_lower2
    red_range_upper1 = list(map(int, table2.getValue('String 4', Any).split(','))) #red_range_upper1
    red_range_upper2 = list(map(int, table2.getValue('String 5', Any).split(','))) #red_range_upper2

    lower = {'blue':(blue_range_lower[0],blue_range_lower[1],blue_range_lower[2]),'red':(red_range_lower1[0],red_range_lower1[1],red_range_lower1[2])}
    upper = {'blue':(blue_range_upper[0],blue_range_upper[1],blue_range_upper[2]),'red':(red_range_upper1[0],red_range_upper1[1],red_range_upper1[2])}

    lower2 = {'red':(red_range_lower2[0],red_range_lower2[1],red_range_lower2[2])}
    upper2 = {'red':(red_range_upper2[0],red_range_upper2[1],red_range_upper2[2])}
    #print(blue_range_lower)
    
 # --------LIVE CAMERA INPUT END--------
