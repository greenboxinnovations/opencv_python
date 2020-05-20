import cv2
import cv2.aruco as aruco

import numpy as np
from numpy.linalg import norm
 
# ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
# ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

ip_left =  "rtsp://192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
# ip_right = "rtsp://192.168.0.124:554/Streaming/Channels/1/?transportmode=unicast"

# ip_left =   "rtsp://192.168.0.140:8554/live0.264"
# ip_right = "rtsp://192.168.0.141:8554/live0.264"

# ip_left =   0

cap = cv2.VideoCapture(ip_left)

cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image1', 1280, 720)

print("Press q to Quit")
print("Press a to toggle tracking Aruco")
print("Press d to toggle drawing Aruco")
print("Press c to toggle drawing Corner")

trackAruco  = False
drawAruco   = False
drawCorner  = False

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()

ref_frame_axies = []
ref_frame_axies.append((10,10))

b = np.array([(0,0)])
a = np.array(ref_frame_axies)
distance = norm(a-b,axis=1)

# print(distance)
# print(distance.min())
# exit()


def drawCorners(ids, corners, img):
	try:
		for i, id_single in enumerate(ids):
			# print(corners[i][0][0][0])
			p = tuple(corners[i][0][0])
			cv2.circle(img, p, 10, (0,0,255), -1)
	except Exception as e:
		print(e)
	
	return img

while(True):

	try:
		# Capture frame-by-frame
		ret, frame = cap.read()
		#print(frame.shape) #480x640
		# Our operations on the frame come here
		# gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray = frame
		# aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
		
		if trackAruco:
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		

		if trackAruco and drawAruco:
			gray = aruco.drawDetectedMarkers(gray, corners, ids)


		if trackAruco and drawCorner:
			if ids is not None:
				gray = drawCorners(ids, corners, gray)
		# gray = aruco.drawDetectedMarkers(gray, corners, ids)

		# Display the resulting frame
		cv2.imshow('image1',gray)

		k = cv2.waitKey(20)
		if k == ord('q'):
			break

		if k == ord('a'):
			trackAruco = not trackAruco

		if k == ord('d'):
			drawAruco = not drawAruco

		if k == ord('c'):
			drawCorner = not drawCorner
				
	except Exception as e:
		# raise e
		pass
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()