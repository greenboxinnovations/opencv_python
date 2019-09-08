import numpy as np
import cv2
import cv2.aruco as aruco


ip_left =  "rtsp://192.168.0.123:554/Streaming/Channels/1/?transportmode=unicast"
ip_right = "rtsp://192.168.0.124:554/Streaming/Channels/1/?transportmode=unicast"

cap1 = cv2.VideoCapture(ip_left)
cap2 = cv2.VideoCapture(ip_right)

cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
cv2.namedWindow('image2', cv2.WINDOW_NORMAL)

cv2.resizeWindow('image1', 640, 480)
cv2.resizeWindow('image2', 640, 480)


# fs = cv2.FileStorage("cam_123.yml", cv2.FILE_STORAGE_READ)
fs = cv2.FileStorage("cam_123.yml", cv2.FILE_STORAGE_READ)
cm1 = fs.getNode("camera_matrix").mat()
dc1 = fs.getNode("distortion_coefficients").mat()


# fs = cv2.FileStorage("cam_123.yml", cv2.FILE_STORAGE_READ)
fs = cv2.FileStorage("cam_124.yml", cv2.FILE_STORAGE_READ)
cm2 = fs.getNode("camera_matrix").mat()
dc2 = fs.getNode("distortion_coefficients").mat()



fs = cv2.FileStorage("extrinsics.yml", cv2.FILE_STORAGE_READ)
P1 = fs.getNode("P1").mat()
P2 = fs.getNode("P2").mat()

R = fs.getNode("R").mat()
T = fs.getNode("T").mat()

# print(R)
# print(T)
# print(P2)
# exit()



aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()
g=0

while(True):
    # Capture frame-by-frame
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    # Our operations on the frame come here
    # gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    # gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    gray1 = frame1
    gray2 = frame2



    corners1, ids1, rejectedImgPoints1 = aruco.detectMarkers(gray1, aruco_dict, parameters=parameters)
    corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(gray2, aruco_dict, parameters=parameters)


    gray1 = aruco.drawDetectedMarkers(gray1, corners1)
    gray2 = aruco.drawDetectedMarkers(gray2, corners2)

    try:
        ud1 = cv2.undistortPoints(corners1[0], cm1, dc1)
        ud2 = cv2.undistortPoints(corners2[0], cm2, dc2)    

        if g==0:
            g=g+1
            
            

            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners1[0], 0.158, cm1, dc1)            

            rotation_mat = np.zeros(shape=(3, 3))
            R1 = cv2.Rodrigues(rvecs[0], rotation_mat)[0]
            T1 = np.transpose(tvecs[0])
            # Pj = np.column_stack((np.matmul(cm1,Rj), tvecs[0]))


            R2 = np.matmul(R,R1)
            T2 = np.matmul(R,T1) + T
            # print(T2)            
            # print(np.dot(cm1,Rj))
            # print(np.dot(cm1,tvecs[0]))

            # print(R1)
            # print(T1)


            P1_new = np.zeros(shape=(3, 4))
            rot_trans_mat1 = np.zeros(shape=(3, 4))
            rot_trans_mat1 = np.concatenate((R1, T1), axis=1)            
            P1_new = np.matmul(cm1,rot_trans_mat1)

            P2_new = np.zeros(shape=(3, 4))
            rot_trans_mat2 = np.zeros(shape=(3, 4))
            rot_trans_mat2 = np.concatenate((R2, T2), axis=1)            
            P2_new = np.matmul(cm2,rot_trans_mat2)

            # print(P1_new)
            # print(P2_new)

            

            # points4D = cv2.triangulatePoints(P1_new, P2_new, ud1, ud2)
            points4D = cv2.triangulatePoints(P1, P2, ud1, ud2)
            points3D = cv2.convertPointsFromHomogeneous(points4D)
            print(points3D)
            # exit()

    except Exception as e:
        # pass
        print(e)
        # exit()
    # print(corners2[0])
    






    # Display the resulting frame
    cv2.imshow('image1',gray1)
    cv2.imshow('image2',gray2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap1.release()
cap2.release()
cv2.destroyAllWindows()