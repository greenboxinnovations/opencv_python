import cv2
import cv2.aruco as aruco
import numpy as np
 
ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

cap = cv2.VideoCapture(ip_right)


# cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
# cv2.namedWindow('image2', cv2.WINDOW_NORMAL)

# cv2.resizeWindow('image1', 640, 480)
# cv2.resizeWindow('image2', 640, 480)

# aruco init
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()


fs = cv2.FileStorage("intrinsics.yml", cv2.FILE_STORAGE_READ)
camera_matrix = fs.getNode("M2").mat()
distortion_coefficients = fs.getNode("D2").mat()
 

markerLength = 0.155
axis_length = 0.1

while(True):

    try:
        # Capture frame-by-frame
        ret, frame = cap.read()
        gray = frame

        # detect markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        rvecs, tvecs, _objPoints    =   cv2.aruco.estimatePoseSingleMarkers( corners, markerLength, camera_matrix, distortion_coefficients)
        # print(tvecs)
     
        # draw makers
        gray = aruco.drawDetectedMarkers(gray, corners)
        # draw axis
        gray = aruco.drawAxis(gray, camera_matrix, distortion_coefficients, rvecs, tvecs, axis_length)


        rotation_mat = np.zeros(shape=(3, 3))
        R = cv2.Rodrigues(rvecs[0], rotation_mat)[0]      

        # print(R)
        # print(tvecs[0])
        # print(tvecs[0].T)
        Rt = np.hstack((R,tvecs[0].T))
        # print(Rt)       

        P = np.matmul(camera_matrix,Rt)

        print(P)



        rotation_mat1 = np.zeros(shape=(3, 3))
        R1 = cv2.Rodrigues(rvecs[0], rotation_mat1)[0]        
        P1_n = np.column_stack((np.matmul(camera_matrix,R1), tvecs[0].T))  
        print(P1_n)


        exit() 


     
        # show results
        cv2.imshow('frame',gray)
    except Exception as e:
        raise e
        # pass

    
    # UI loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()