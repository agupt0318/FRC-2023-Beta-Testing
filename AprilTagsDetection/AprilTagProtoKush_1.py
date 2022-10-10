import cv2
import time
import math
import logging
import threading
from pupil_apriltags import Detector

# error value for calculating FPS
epsilon = 0.0000001

def AprilTag_Detction():

    at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
    )

    while True:

        starttime = time.time()

        frame = camera.read()[1]
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        res = at_detector.detect(gray)

        for i in res:

            (ptA, ptB, ptC, ptD) = i.corners
            ptA = (int(ptA[0]), int(ptA[1]))
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))

            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

            (cX, cY) = (int(i.center[0]), int(i.center[1]))
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            tagID = str(i.tag_id)
            cv2.putText(frame, tagID, (cX - 10, cY - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)

        print(res)

        fps_int = round(1.0/(time.time()-starttime+epsilon),2)
        fps = "FPS: "+str(fps_int)
        cv2.putText(frame,fps,(60,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0))[2]

        cv2.imshow("Frame: ",frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    
    logging.basicConfig(level=logging.DEBUG)
    
    camera = cv2.VideoCapture(0)
    AprilTag_Detction()
