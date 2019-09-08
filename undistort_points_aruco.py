import cv2
import cv2.aruco as aruco
import numpy as np
 
ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

cap = cv2.VideoCapture(ip_right)

p = False


# fs = cv2.FileStorage("cam_123.yml", cv2.FILE_STORAGE_READ)
# fs = cv2.FileStorage("cam_124.yml", cv2.FILE_STORAGE_READ)
# camera_matrix = fs.getNode("camera_matrix").mat()
# distortion_coefficients = fs.getNode("distortion_coefficients").mat()
fs = cv2.FileStorage("intrinsics.yml", cv2.FILE_STORAGE_READ)
camera_matrix = fs.getNode("M2").mat()
distortion_coefficients = fs.getNode("D2").mat()

dist = True
 
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = frame
    # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters =  aruco.DetectorParameters_create()
 
    #print(parameters)
 
    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


    # corners32 = np.array(corners,dtype=np.float32)
    # pts_uv = cv2.undistortPoints(corners32, camera_matrix, distortion_coefficients)
    # corners32 = np.asarray(corners, dtype=np.float32)

    if not p:
        p = True
        print(corners[0])
        # print(corners)

        # for x in np.nditer(corners):
        #     print(x)


        # corners1 = corners[0].reshape(-1, 2)
        # print(corners1)
    # if not p:
    # 	p = True
    # 	# print(type(corners))
    # 	print(corners)
    # 	# camera_matrix = np.array([[1.3e+03, 0., 6.0e+02], [0., 1.3e+03, 4.8e+02], [0., 0., 1.]], dtype=np.float32)
    # 	# dist_coeffs = np.array([-2.4-01, 9.5e-02, -4.0e-04, 8.9e-05, 0.], dtype=np.float32)
        # test = np.zeros((10,1,2), dtype=np.float32)
        # print(test)
        xy_undistorted = cv2.undistortPoints(corners[0], camera_matrix, distortion_coefficients, P=camera_matrix)
        print(xy_undistorted)

        # jiggy = np.zeros_like(corners[0],dtype=np.float32)
        # cv2.fisheye.distortPoints(xy_undistorted,jiggy,camera_matrix, distortion_coefficients)
        # print(type(jiggy))
 
    #It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs
 
    gray = aruco.drawDetectedMarkers(gray, corners)

    
    if dist:
    	dst	= cv2.undistort(gray, camera_matrix, distortion_coefficients)
    else:
    	dst = gray
 

    try:
    	cv2.circle(dst, tuple(corners[0][0][0]), 5, (0,0,255), -1)
    	# cv2.circle(dst, tuple(pts_uv[0][0][0]), 5, (0,255,0), -1)
    except Exception as e:
    	pass
    

    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame',dst)
    k = cv2.waitKey(33)
    if k == ord('q'):
    	break

    if k == ord('d'):
    	dist = not dist
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()