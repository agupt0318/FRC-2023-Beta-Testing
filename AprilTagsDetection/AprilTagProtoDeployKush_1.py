# Imports
import cv2
import time
import math # might be used
import logging
import threading

from pupil_apriltags import Detector
from networktables import NetworkTables as nt
from cscore import CameraServer

# CONSTANTS

# camera resolution settings TODO: Recalculate width and height constants
height = 180 # 720 / 180 / 225
width = 320 # 1280 / 320 / 400

# error value for calculating FPS
epsilon = 0.0000001

# Table update function
def table_update(table,store):
    
    # loop till the penultimate element and push that to nt
    for i in range(len(store)-1):
        table.putNumber(f"Tag {i} content: ",store[i])

    # push the fps number to nt
    table.putNumber("FPS: ",store[-1])

# here when needed to translate tag id's into actions, perhaps
def translator(tag_id):
    pass

def AprilTag_Detction():

    # Define the tag detector
    at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
    )

    # Infinite loop in which the program runs
    while True:

        # time the iteration
        starttime = time.time()

        # take one input from the camera
        frame = camera.read()[1]
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        # Detect tags in the grayscale frame
        res = at_detector.detect(gray)

        # store tag information computed later
        tags = []

        # loop through the tags detected in frame
        for i in res:

            # Extract the corners of the tag
            (ptA, ptB, ptC, ptD) = i.corners
            ptA = (int(ptA[0]), int(ptA[1]))
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))

            # Draw the tag bounding box on the frame
            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

            # Draw a circle at the center of the tag
            (cX, cY) = (int(i.center[0]), int(i.center[1]))
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            # Extract the id of the current tag and draw it near the center
            tagID = str(i.tag_id)
            cv2.putText(frame, tagID, (cX - 10, cY - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)

            # Add the tag id to to the tags list
            tags.append(tagID)

        # Compute FPS
        fps_int = round(1.0/(time.time()-starttime+epsilon),2)

        # fps = "FPS: "+str(fps_int)
        # cv2.putText(frame,fps,(60,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0))[2]

        # Append the FPS to the end of tags
        tags.append(fps_int)

        # Update nt with the most current information
        table_update(table,tags)

        # Output the edited frame to nt, with all the tags highlighted and decoded
        outputStream.putFrame(frame)

if __name__ == '__main__':

    # Configure logging to see errors and warnings
    logging.basicConfig(level=logging.DEBUG)

    # Intizalize a condition to notify a thread
    cond = threading.Condition()
    notified = [False]
    
    # Define the connection listner to connect to the rPi
    def connectionListener(connected, info):
    
        print(info, '; Connected=%s' % connected)
        with cond:
            notified[0] = True
            cond.notify()
    
    # Initialize connection to the rPi
    nt.initialize(server="10.26.38.2")

    # Listen for a connection
    nt.addConnectionListener(connectionListener, immediateNotify=True)
    
    # Print waiting while not connected
    with cond:
        print("Waiting")
        if not notified[0]:
            cond.wait()

    # Print connected when successfully connected
    print("Connected")

    # Define the output table
    table = nt.getTable('Tracked Tags')

    # Get an instance of CameraServer
    cs = CameraServer.getInstance()
    
    # Set up a video stream using opencv
    camera = cv2.VideoCapture(0)

    # Camera settings ie width, height and brightness.
    camera.set(3,width)
    camera.set(4,height)
    camera.set(10,0.45)
    
    # This will send images back to the Dashboard.
    outputStream = cs.putVideo("Tags Detected", width, height)

    # Call the detection function
    AprilTag_Detction()
