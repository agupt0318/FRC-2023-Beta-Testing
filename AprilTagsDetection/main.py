from pupil_apriltags import Detector
import cv2
import numpy as np
import time


'''
# Camera calibration and distortion parameters (OpenCV)
Camera.fx: 1394.6027293299926
Camera.fy: 1394.6027293299926
Camera.cx: 995.588675691456
Camera.cy: 599.3212928484164

'''


def draw_pose(overlay, camera_params, tag_size, pose_R, pose_T, z_sign=1):
   opoints = np.array([
      -1, -1, 0,
      1, -1, 0,
      1, 1, 0,
      1, -1, -2 * z_sign,
   ]).reshape(-1, 1, 3) * 0.5 * tag_size

   fx, fy, cx, cy = camera_params

   K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

   rvec = pose_R
   tvec = pose_T

   dcoeffs = np.zeros(5)

   ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

   ipoints = np.round(ipoints).astype(int)

   ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

   cv2.line(overlay, ipoints[0], ipoints[1], (0, 0, 255), 2)
   cv2.line(overlay, ipoints[1], ipoints[2], (0, 255, 0), 2)
   cv2.line(overlay, ipoints[1], ipoints[3], (255, 0, 0), 2)
   font = cv2.FONT_HERSHEY_SIMPLEX
   cv2.putText(overlay, 'X', ipoints[0], font, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
   cv2.putText(overlay, 'Y', ipoints[2], font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
   cv2.putText(overlay, 'Z', ipoints[3], font, 0.5, (255, 0, 0), 2, cv2.LINE_AA)


camera_params = [1394.6027293299926, 1394.6027293299926, 995.588675691456, 599.3212928484164]
camera_params2 = np.array([[1394.6027293299920, 0, 995.588675691456], [0, 1394.6027293299926, 599.3212928484164], [0, 0, 1]]).reshape(3, 3)
# camera_params = (
#    camera_params2[0, 0],
#    camera_params2[1, 1],
#    camera_params2[0, 2],
#    camera_params2[1, 2]
# )
print(type(camera_params))
at_detector = Detector(
   families="tag36h11",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)

vid = cv2.VideoCapture(0)
epsilon = 0.0000001

vid.set(3, 640)
vid.set(4, 480)
while (True):

   ret, frame = vid.read() # read video frame
   starttime = time.time()

   # detect april tags with their library
   img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   #results = at_detector.detect(img)
   # pose estimations:
   results = at_detector.detect(img, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.206375)

   print(results)
   # results outputs all april tags found -> loop through them
   for r in results:
      # find coordinates for each april tag found
      (ptA, ptB, ptC, ptD) = r.corners
      ptA = (int(ptA[0]), int(ptA[1]))
      ptB = (int(ptB[0]), int(ptB[1]))
      ptC = (int(ptC[0]), int(ptC[1]))
      ptD = (int(ptD[0]), int(ptD[1]))

      # draw box around april tags -> don't use cv2.rect just in case
      # we are looking at an angle and the box doesn't have 100% cover ration
      cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
      cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
      cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
      cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

      # put point on center of april tag
      (cX, cY) = (int(r.center[0]), int(r.center[1]))
      cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
      print(r.pose_R.shape)
      draw_pose(frame, camera_params, tag_size=0.206375, pose_R=r.pose_R, pose_T=r.pose_t, z_sign=1)
      print(r)
      # find what id the tag has and save it + display
      tagID = str(r.tag_id)
      cv2.putText(frame, tagID, (ptA[0], ptA[1] - 50),
                  cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)

   fps_int = round(1.0 / (time.time() - starttime + epsilon), 2)
   fps = "FPS: " + str(fps_int)
   cv2.putText(frame, fps, (60, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0))[2]
   # Display the resulting frame
   cv2.imshow('frame', frame)

   if cv2.waitKey(1) & 0xFF == ord('q'):
      break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()


