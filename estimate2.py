import cv2
import cv2.aruco as aruco
import numpy as np
 
ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

cap1 = cv2.VideoCapture(ip_left)
cap2 = cv2.VideoCapture(ip_right)


cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
cv2.namedWindow('image2', cv2.WINDOW_NORMAL)

cv2.resizeWindow('image1', 640, 480)
cv2.resizeWindow('image2', 640, 480)

# aruco init
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()


fs = cv2.FileStorage("intrinsics.yml", cv2.FILE_STORAGE_READ)
cm1 = fs.getNode("M1").mat()
cm2 = fs.getNode("M2").mat()
dc1 = fs.getNode("D1").mat()
dc2 = fs.getNode("D2").mat()



fs2 = cv2.FileStorage("extrinsics.yml", cv2.FILE_STORAGE_READ)
P1 = fs2.getNode("P1").mat()
P2 = fs2.getNode("P2").mat()
# print(P2.shape)
# print(P2)
 

dist = False

markerLength = 0.155
axis_length = 0.1

while(True):

    try:
        # Capture frame-by-frame
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        gray1 = frame1
        gray2 = frame2

        # detect markers
        corners1, ids1, rejectedImgPoints1 = aruco.detectMarkers(gray1, aruco_dict, parameters=parameters)
        corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(gray2, aruco_dict, parameters=parameters)
        
        rvecs1, tvecs1, _objPoints1    =   cv2.aruco.estimatePoseSingleMarkers( corners1, markerLength, cm1, dc1)
        rvecs2, tvecs2, _objPoints2    =   cv2.aruco.estimatePoseSingleMarkers( corners2, markerLength, cm2, dc2)
        # print(tvecs)
     
        # draw makers
        gray1 = aruco.drawDetectedMarkers(gray1, corners1)
        gray2 = aruco.drawDetectedMarkers(gray2, corners2)
        # # draw axis
        # gray1 = aruco.drawAxis(gray1, cm1, dc1, rvecs1, tvecs1, axis_length)
        # gray2 = aruco.drawAxis(gray2, cm2, dc2, rvecs2, tvecs2, axis_length)




        # rotation_mat1 = np.zeros(shape=(3, 3))
        # R1 = cv2.Rodrigues(rvecs1[0], rotation_mat1)[0]        
        # P1_n = np.column_stack((np.matmul(cm1,R1), tvecs1[0].T))        

        # rotation_mat2 = np.zeros(shape=(3, 3))
        # R2 = cv2.Rodrigues(rvecs2[0], rotation_mat2)[0]        
        # P2_n = np.column_stack((np.matmul(cm2,R2), tvecs2[0].T))     


        rotation_mat1 = np.zeros(shape=(3, 3))
        R1 = cv2.Rodrigues(rvecs1[0], rotation_mat1)[0]        
        RT1 = np.hstack((R1,tvecs1[0].T))  
        P1_n = np.matmul(cm1,RT1)   

        rotation_mat2 = np.zeros(shape=(3, 3))
        R2 = cv2.Rodrigues(rvecs2[0], rotation_mat2)[0]        
        RT2 = np.hstack((R2,tvecs2[0].T))  
        P2_n = np.matmul(cm2,RT2)

        axisPoints = np.float32([
            [0, 0, 0],
            [0.075, 0.075, 0],
            [-0.075, -0.075, 0], 
            [0.075, -0.075, 0], 
            [-0.075, 0.075, 0]]
            ).reshape(-1, 3)
        # print(axisPoints)
        # exit()
        axisImgPoints1, jac = cv2.projectPoints(axisPoints, rvecs1, tvecs1, cm1, dc1)

        if dist:
            gray1 = cv2.undistort(gray1, cm1, dc1)
            axisImgPoints1_ud = cv2.undistortPoints(axisImgPoints1, cm1, dc1, P=cm1)
            # cv2.circle(gray2,tuple(axisImgPoints2[0].ravel()), 3, (0,255,0), -1)
            cv2.circle(gray1,tuple(axisImgPoints1_ud[1].ravel()), 3, (0,0,255), -1)
            cv2.circle(gray1,tuple(axisImgPoints1_ud[2].ravel()), 3, (0,0,255), -1)
            cv2.circle(gray1,tuple(axisImgPoints1_ud[3].ravel()), 3, (0,0,255), -1)
            cv2.circle(gray1,tuple(axisImgPoints1_ud[4].ravel()), 3, (0,0,255), -1)

        cv2.circle(gray1,tuple(axisImgPoints1[0].ravel()), 1, (0,255,0), -1)
        cv2.circle(gray1,tuple(axisImgPoints1[1].ravel()), 1, (255,255,0), -1)
        cv2.circle(gray1,tuple(axisImgPoints1[2].ravel()), 1, (255,255,0), -1)
        cv2.circle(gray1,tuple(axisImgPoints1[3].ravel()), 1, (255,255,0), -1)
        cv2.circle(gray1,tuple(axisImgPoints1[4].ravel()), 1, (255,255,0), -1)



        axisImgPoints2, jac = cv2.projectPoints(axisPoints, rvecs2, tvecs2, cm2, dc2)

        if dist:
            gray2 = cv2.undistort(gray2, cm2, dc2)
            axisImgPoints2_ud = cv2.undistortPoints(axisImgPoints2, cm2, dc2, P=cm2)
            # print(type(axisImgPoints2_ud))
            # print(axisImgPoints2_ud)

            # cv2.circle(gray2,tuple(axisImgPoints2[0].ravel()), 3, (0,255,0), -1)
            cv2.circle(gray2,tuple(axisImgPoints2_ud[1].ravel()), 3, (0,0,255), -1)
            cv2.circle(gray2,tuple(axisImgPoints2_ud[2].ravel()), 3, (0,0,255), -1)
            cv2.circle(gray2,tuple(axisImgPoints2_ud[3].ravel()), 3, (0,0,255), -1)
            cv2.circle(gray2,tuple(axisImgPoints2_ud[4].ravel()), 3, (0,0,255), -1)

        cv2.circle(gray2,tuple(axisImgPoints2[0].ravel()), 1, (0,255,0), -1)
        cv2.circle(gray2,tuple(axisImgPoints2[1].ravel()), 1, (255,255,0), -1)
        cv2.circle(gray2,tuple(axisImgPoints2[2].ravel()), 1, (255,255,0), -1)
        cv2.circle(gray2,tuple(axisImgPoints2[3].ravel()), 1, (255,255,0), -1)
        cv2.circle(gray2,tuple(axisImgPoints2[4].ravel()), 1, (255,255,0), -1)


        # gray2 = cv2.line(gray1, tuple(), tuple(imgPoints[0].ravel()), (255, 0, 0), 5)
        # gray2 = cv2.line(gray1, tuple, tuple(imgPoints[1].ravel()), (0, 255, 0), 5)

        # draw axis
        # gray1 = aruco.drawAxis(gray1, cm1, dc1, rvecs1, tvecs1, axis_length)
        # gray2 = aruco.drawAxis(gray2, cm2, dc2, rvecs2, tvecs2, axis_length)

        if dist:

            # this fooking works
            points4D = cv2.triangulatePoints(P1_n, P2_n, axisImgPoints1_ud, axisImgPoints2_ud)
            points3D = cv2.convertPointsFromHomogeneous(points4D.T)
            np.set_printoptions(suppress=True)
            np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
            print(points3D)
            print("")
            print("")

            corners1_ud = cv2.undistortPoints(corners1[0], cm1, dc1, P=cm1)
            corners2_ud = cv2.undistortPoints(corners2[0], cm2, dc2, P=cm2)

            points4D = cv2.triangulatePoints(P1_n, P2_n, corners1_ud, corners2_ud)
            points3D = cv2.convertPointsFromHomogeneous(points4D.T)            
            print(points3D)
            # exit()
            dist = not dist

            # exit()
            # print(axisImgPoints1_ud.dtype)


            # print(P1)
            # print(P1_n)

            # float32
            # print(corners1[0].dtype)
            # print(axisImgPoints1.dtype)

            # ndarray float32

            # points4D = cv2.triangulatePoints(P1, P2, corners1[0], corners2[0])            
            # points3D = cv2.convertPointsFromHomogeneous(points4D)
            # print(points3D)
            # exit()
     
        # show results
        cv2.imshow('image1',gray1)
        cv2.imshow('image2',gray2)
    except Exception as e:
        # raise e
        pass

    
    # # UI loop
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break


    k = cv2.waitKey(33)
    if k == ord('q'):
        break

    if k == ord('d'):
        dist = not dist
 
# When everything done, release the capture
cap1.release()
cap2.release()
cv2.destroyAllWindows()