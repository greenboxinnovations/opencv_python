import numpy as np
import cv2


# ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
# ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

# ip_left =  "rtsp://192.168.0.123:554/Streaming/Channels/1/?transportmode=unicast"
# ip_right = "rtsp://192.168.0.124:554/Streaming/Channels/1/?transportmode=unicast"

ip_left =   "rtsp://192.168.0.140:8554/live0.264"
ip_right = "rtsp://192.168.0.141:8554/live0.264"

cap = cv2.VideoCapture(ip_right)

cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image1', 1280, 720)



# fs = cv2.FileStorage("cam_123.yml", cv2.FILE_STORAGE_READ)
# fs = cv2.FileStorage("cam_124.yml", cv2.FILE_STORAGE_READ)
# fs = cv2.FileStorage("cam_140.yml", cv2.FILE_STORAGE_READ)
fs = cv2.FileStorage("cam_141.yml", cv2.FILE_STORAGE_READ)
camera_matrix = fs.getNode("camera_matrix").mat()
distortion_coefficients = fs.getNode("distortion_coefficients").mat()
# camera_matrix = fs.getNode("M1").mat()
# distortion_coefficients = fs.getNode("D1").mat()


dist = True

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    if ret:


	    # Our operations on the frame come here
	    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


	    if dist:
	    	dst	= cv2.undistort(gray, camera_matrix, distortion_coefficients)
	    else:
	    	dst = gray


	    cv2.imshow('image1',dst)	    

	    k = cv2.waitKey(33)
	    if k == ord('q'):
	    	break

	    if k == ord('d'):
	    	dist = not dist

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()