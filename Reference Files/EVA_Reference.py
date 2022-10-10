import threading
import math
import logging
import cv2
import time

from networktables import NetworkTables as nt
from cscore import CameraServer

"""
PRODUCTION VERSION
threshold_counturs.py: Thresholds an image and finds contours and detects balls for FRC 2022
    inspired from: https://github.com/rebels2638/frc-vision-2022/blob/main/countour_threshold_method/threshold_contours.py, https://stackoverflow.com/questions/58016325/thinness-ratio-calculation-wrong-results?noredirect=1&lq=1

The UrMom Alliance consists of urMom Caden Li, and urMom Kushagra Saxena
Rankings:
urMom: (Founders)
urSister: (Leaders)
urAunt: (lowlies)
"""
 
__author__ = ["UrMom Alliance"]
__copyright__ = ["Copyright 2022, Caden Li & Kushagra Saxena"]
__credits__ = ["Collin Li"]
__license__ = "MIT"

    # ---------- CONSTANTS ----------

# colors for drawing circles in BGR
colors = {'Blue':(255,0,0),'Red':(0,0,255)}

# 2 sets of lower and upper values for debuging purposes. Both work in real-world testing

# red show only, upper and lower range (HSV)
# lower = {'Blue':(90,100,35),'Red':(0,90,150)} 
# upper = {'Blue':(130,255,255),'Red':(10,255,255)}

# blue show only, upper and lower range (HSV)
lower = {'Red':(0,89,40),'Blue':(90,126,30)} 
upper = {'Red':(10,255,255),'Blue':(130,255,255)} 

# second red range for complete masking (HSV)
lower2 = {'Red':(170,102,50)}
upper2 = {'Red':(180,255,255)}

# constants for distance calculating
known_distance = 314.9 # real ball distance in cm
known_width = 24.13 # real ball width in cm
ref_image_ball_width = 15.6*2 # pixel width of ball at 193CM

# camera resolution settings
height = 180 # 720 / 180 / 225
width = 320 # 1280 / 320 / 400

# height of cam from the ground
cam_height = 69.420 # 69.850

# error value for calculating FPS
epsilon = 0.0000001

# function which decides how to networktables based on provided inputs
def table_update(table,store,color):

    # [[x,y,radius,sti,angle,retry,key]] if store is non-empty
    # store is empty when no color in frame []

    if len(store) == 0: # if a singular color is not detected

        table.putBoolean(f"{color}_Seeking",False)
        
    else: # if a color is detected
 
        table.putNumber(f"{color}_Certainty", float(store[3]))
        table.putNumber(f"{color}_Distance", float(store[5]))
        table.putNumber(f"{color}_Angle", float(store[4]))
        table.putBoolean(f"{color}_Seeking",True)

# Ball detection function 
def ballDetection():

    # while true to grab frames in a live video feed
    while True:
        
        # grab a frame from the video feed
        frame = camera.read()[1]

        # start timing the iteration
        starttime = time.time()
     
        # blurred = cv2.GaussianBlur(frame, (9,9), 0)

        # switch color spaces from BGR (Default) to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # area and perimeter definitions to solve scope issues
        area = 0
        perimeter = 1

        # arrays which store contours of a specified color, max len == 1
        red_store = []
        blue_store = []

        # each iteration finds contours for a specfic color in colors list (current len is 2)
        for key in upper.keys():
            
            # temporary storage
            tempStore = []

            # build a mask for the current color
            mask = cv2.inRange(hsv, lower[key], upper[key])

            # if the color is red, apply additional modifications to the mask
            if(key == "Red"):
                mask += cv2.inRange(hsv, lower2[key], upper2[key])

            # use morphology to erode noise in the image
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (5,5), iterations = 2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, (3,3), iterations = 4)
            
            # find the contours in the binary image
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

            # used in the inner for loop to find contour with least distance
            retry = 999999

            # start iterating through the contours in the list
            for i in cnts:
               
                # calculate characteristics of the contour ie area, circumfrence, center coordinates and radius
                ((x, y), radius) = cv2.minEnclosingCircle(i)
                area = cv2.contourArea(i)
                perimeter = cv2.arcLength(i, True)

                # If there is a valid perimeter and area, then find the % accuracy.
                if perimeter != 0:

                    # compares the area of the contour and find the percent it covers on the min enclosing circle (circumcircle)
                    circleArea = radius**2 * 3.14159; # finds the area of the min enclosing circle
                    ti = round((4*area*math.pi)/(perimeter**2),2) #area/circleArea
                    sti = str(ti)
  
                if radius > 10 and ti > 0.65 and y > 40: # replace radius for 400pi (r = 20), TODO: tinker with y value based on camera angle

                    # finds distance from ball to camera(hypotenuse), and then uses pythagorean theorem to find the distance to the robot instead (leg)
                    camDist = round(Distance_Finder(Focal_length_found,known_width,radius*2),2)
                    Distance = str(round(math.sqrt(abs((camDist**2)-(cam_height**2))),2))
                    
                    # finds angle offset of the ball based on its x location relative to the total width
                    angle = str(round(((x-(width/2))/(width))*53,2)) # was 22.5, total lifecam FOV is 53 degrees (hor.) +-26.5

                    # if there is a ball closer than the closest one so far, we update it.
                    if float(Distance) < float(retry): # change Distance to dist, also in the loop
 
                        retry = Distance
                        tempStore.append([x,y,radius,sti,angle,retry,key]) # x - width/2 was angle or the other way around

            # if tempStore has any ball stored, we will check its color, and take in the cloest(last index) ball
            if tempStore != []:

                if key == 'Red': # fixed red to Red
                    red_store.append(tempStore[len(tempStore)-1])

                else:
                    blue_store.append(tempStore[len(tempStore)-1]) # -1

        # blue_store.append(False)
        # red_store.append(False)

        # if blue contours are present then draw the contour then send its information to nt
        if blue_store != []:
            
            cv2.circle(frame, (int(blue_store[0]), int(blue_store[1])), int(blue_store[2]), colors['Blue'], 3) # red
            cv2.circle(frame, (int(blue_store[0]), int(blue_store[1])), 1, colors['Blue'], 3)

        # if red contours are present then draw the contour then send its information to nt
        if red_store != []:
 
            cv2.circle(frame, (int(red_store[0]), int(red_store[1])), int(red_store[2]), colors['Red'], 3) # red
            cv2.circle(frame, (int(red_store[0]), int(red_store[1])), 1, colors['Red'], 3)

        table_update(table,red_store,'Red')
        table_update(table,blue_store,'Blue')

        # calculate the fps based on starttime
        fps_int = round(1.0/(time.time()-starttime+epsilon),2)

        # put fps on nt
        table.putNumber("fps", fps_int)

        # display the frame
        outputStream.putFrame(frame)

if __name__ == '__main__':

    # configure logging to see errors and warnings
    logging.basicConfig(level=logging.DEBUG)

    # intizalize a condition to notify a thread
    cond = threading.Condition()
    notified = [False]
    
    # define the connection listner to connect to the rPi
    def connectionListener(connected, info):
    
        print(info, '; Connected=%s' % connected)
        with cond:
            notified[0] = True
            cond.notify()
    
    # initialize connection to the rPi
    nt.initialize(server="10.26.38.2")

    # listen for a connection
    nt.addConnectionListener(connectionListener, immediateNotify=True)
    
    # print waiting while not connected
    with cond:
        print("Waiting")
        if not notified[0]:
            cond.wait()

    # print connected when successfully connected
    print("Connected")

    # get the table to update
    table = nt.getTable('Tracked Balls') # SmartDashboard

    # set the uodate rate of nt to 18ms
    nt.setUpdateRate(0.018)

    # finds the focal length of the camera
    def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):
        return (width_in_rf_image*measured_distance)/real_width

    # uses focal length to return distance from camera to object
    def Distance_Finder(Focal_length,real_ball_width,ball_width_in_frame):
        return (real_ball_width*Focal_length)/ball_width_in_frame
    
    # calculate the focal length
    Focal_length_found = Focal_Length_Finder(known_distance,known_width,ref_image_ball_width)
    
    # get an instance of CameraServer
    cs = CameraServer.getInstance()
    
    # set up a video stream using opencv
    camera = cv2.VideoCapture(-1)

    # camera settings ie width, height and brightness
    camera.set(3,width)
    camera.set(4,height)
    camera.set(10,0.45) # TODO: change constance EVA
    
    # This will send images back to the Dashboard
    outputStream = cs.putVideo("Balls Detected", width, height)

    # call the Ball Detection function
    ballDetection()