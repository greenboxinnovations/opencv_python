import numpy as np
import cv2
import cv2.aruco as aruco

ip_left =  "rtsp://192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
ip_right = "rtsp://192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

cap = cv2.VideoCapture(ip_left)

p = False

# fs = cv2.FileStorage("cam_123.yml", cv2.FILE_STORAGE_READ)
# fs = cv2.FileStorage("cam_124.yml", cv2.FILE_STORAGE_READ)
fs = cv2.FileStorage("intrinsics.yml", cv2.FILE_STORAGE_READ)
# camera_matrix = fs.getNode("camera_matrix").mat()
# distortion_coefficients = fs.getNode("distortion_coefficients").mat()
camera_matrix = fs.getNode("M1").mat()
distortion_coefficients = fs.getNode("D1").mat()

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()

dist = True

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    if ret:


        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        if dist:
            dst = cv2.undistort(gray, camera_matrix, distortion_coefficients)                        

        else:
            dst = gray

        if not p:
            corners, ids, rejectedImgPoints = aruco.detectMarkers(dst, aruco_dict, parameters=parameters)
            p = True
            print(corners[0])



        cv2.imshow('frame',dst)     

        k = cv2.waitKey(33)
        if k == ord('q'):
            break

        if k == ord('d'):
            p = False
            dist = not dist

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()