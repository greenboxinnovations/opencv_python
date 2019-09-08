import numpy as np
import cv2
import cv2.aruco as aruco


ip_left =  "rtsp://192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
ip_right = "rtsp://192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

cap1 = cv2.VideoCapture(ip_left)
cap2 = cv2.VideoCapture(ip_right)

cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
cv2.namedWindow('image2', cv2.WINDOW_NORMAL)

cv2.resizeWindow('image1', 640, 480)
cv2.resizeWindow('image2', 640, 480)



fs = cv2.FileStorage("intrinsics.yml", cv2.FILE_STORAGE_READ)
cm1 = fs.getNode("M1").mat()
dc1 = fs.getNode("D1").mat()

cm2 = fs.getNode("M2").mat()
dc2 = fs.getNode("D2").mat()



fs = cv2.FileStorage("extrinsics.yml", cv2.FILE_STORAGE_READ)
P1 = fs.getNode("P1").mat()
P2 = fs.getNode("P2").mat()

R = fs.getNode("R").mat()
T = fs.getNode("T").mat()


# print(R)
# print(T)
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

    dd1 = cv2.undistort(frame1, cm1, dc1)
    dd2 = cv2.undistort(frame2, cm2, dc2)


    corners1, ids1, rejectedImgPoints1 = aruco.detectMarkers(frame1, aruco_dict, parameters=parameters)
    corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(frame2, aruco_dict, parameters=parameters)


    ud1, ids11, rejectedImgPoints11 = aruco.detectMarkers(dd1, aruco_dict, parameters=parameters)
    ud2, ids22, rejectedImgPoints22 = aruco.detectMarkers(dd2, aruco_dict, parameters=parameters)


    gray1 = aruco.drawDetectedMarkers(frame1, corners1)
    gray2 = aruco.drawDetectedMarkers(frame2, corners2)

    # print(corners2[0])
    # print(ud2[0])
    # exit()


    try:        

        if g==0:
            g=g+1
            points4D = cv2.triangulatePoints(P1, P2, ud1[0], ud2[0])
            # print(points4D)


            points3D = cv2.convertPointsFromHomogeneous(points4D)
            # print(points3D)
            # exit()
            

            # projectedPoints = P1.dot(points4D)
            # print(projectedPoints)

            # cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles = cv2.decomposeProjectionMatrix(P1)
            # print(cameraMatrix)
            # exit()

            # imagePoints, jacobian   = cv2.projectPoints(   points3D, rotMatrix, transVect, cameraMatrix, dc1)
            # print(imagePoints)
            # exit()

            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners1[0], 0.158, cm1, dc1)
            # print(rvecs[0])
            # exit()

            # # rotation_mat = np.zeros(shape=(3, 3))
            R1 = cv2.Rodrigues(rvecs[0])[0]
            T1 = np.transpose(tvecs[0])
            # # print(T1)
            # # exit()            
            

            P1_new = np.zeros(shape=(3, 4))
            temp1 = np.zeros(shape=(3, 4))
            temp1 = np.concatenate((R1, T1), axis=1)            

            P1_new = np.matmul(cm1, temp1)
            # # print(P1_new)
            # # exit()



            # # ===================================================================================\
            # # rvecs for the second camera
            rvecs2, tvecs2, _objPoints2 = cv2.aruco.estimatePoseSingleMarkers(corners2[0], 0.158, cm2, dc2)
            R2 = cv2.Rodrigues(rvecs2[0])[0]
            T2 = np.transpose(tvecs2[0])
            # # print(T2)
            # # exit()  

            P2_new = np.zeros(shape=(3, 4))
            temp2 = np.zeros(shape=(3, 4))
            temp2 = np.concatenate((R2, T2), axis=1)            

            P2_new = np.matmul(cm2, temp2)
            # # print(P2_new)
            # # exit()



            R2 = np.matmul(R,R1)
            T2 = np.matmul(R,T1) + T

             
            # R2 = cv2.Rodrigues(rvecs2[0])[0]
            # T2 = np.transpose(tvecs2[0])
            # # print(T2)
            # # exit()  

            P2_new = np.zeros(shape=(3, 4))
            temp2 = np.zeros(shape=(3, 4))
            temp2 = np.concatenate((R2, T2), axis=1)            

            P2_new = np.matmul(cm2, temp2)




            points4D = cv2.triangulatePoints(P1_new, P2_new, ud1[0], ud2[0])
            # # print(points4D)
            # # exit()

            points3D = cv2.convertPointsFromHomogeneous(points4D)
            print(points3D)
            exit()



    except Exception as e:
        # pass
        print(e)
        exit()
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