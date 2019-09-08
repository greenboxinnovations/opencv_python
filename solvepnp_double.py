import cv2
import cv2.aruco as aruco

import numpy as np

import collections
 
ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

cap1 = cv2.VideoCapture(ip_left)
cap2 = cv2.VideoCapture(ip_right)

once = True
 
fs = cv2.FileStorage("intrinsics.yml", cv2.FILE_STORAGE_READ)
cm1 = fs.getNode("M1").mat()
dc1 = fs.getNode("D1").mat()
cm2 = fs.getNode("M2").mat()
dc2 = fs.getNode("D2").mat()

# 0,0               30.5,0
# 
# 
# 
# 0,20              30.5,20
ground = np.float32([
            [0, 0, 0],
            [0.305, 0, 0],
            [0.305, 0.2, 0], 
            [0, 0.2, 0]]
            ).reshape(-1, 3)
# print(ground) 


axis = np.float32([[0.2,0,0], [0,0.2,0], [0,0,-0.2]]).reshape(-1,3)

cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
cv2.namedWindow('image2', cv2.WINDOW_NORMAL)

cv2.resizeWindow('image1', 640, 480)
cv2.resizeWindow('image2', 640, 480)


def draw(img, corners, imgpts):
    # corner = tuple(corners[0].ravel())
    corner = tuple(int(s) for s in corners[0].ravel())        
    img = cv2.line(img, corner, tuple(int(s) for s in imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(int(s) for s in imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(int(s) for s in imgpts[2].ravel()), (0,0,255), 5)
    return img


def makeProjMatrix(cm, rvecs, tvecs):
    rotation_mat = np.zeros(shape=(3, 3))
    R = cv2.Rodrigues(rvecs, rotation_mat)[0]              
    RT = np.hstack((R, tvecs))  
    P = np.matmul(cm, RT)
    return P


def sortArucoById(ids, corners, img):
    # make dictionary to rearrange
    myDict = {}
    id_list = ids.tolist()

    # only first corner from aruco marker
    c_list = []
    for i, id_single in enumerate(ids):


        if id_single != 11:            
            # for plotting
            c_list.append(corners[i][0][0].tolist())            

            # for rearranging
            myDict[id_list[i][0]] = corners[i][0][0].tolist()         

    # plot first corner
    for x, p in enumerate(c_list):
        # print(p)
        # print(tuple(int(s) for s in p))            
        cv2.circle(img,tuple(int(s) for s in p), 10, (0,0,255), -1)


    image_points = np.zeros(shape=(4,2))
    # print(image_points1)

    od = collections.OrderedDict(sorted(myDict.items()))
    # print(od)
    for k, v in od.items(): 
        # print(k, v)
        # np.insert(image_points1,v[0],v[1],axis=0)
        image_points[k-1] = [v[0],v[1]]

    return img, image_points



def targetArucoId(ids, corners):
        
    # image_points = np.zeros(shape=(1,2))    

    for i, id_single in enumerate(ids):
        if id_single == 11:            
            image_points = corners[i]

    return image_points



while(True):

    try:
        # Capture frame-by-frame
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
                
        gray1 = frame1
        gray2 = frame2
        # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()
     

        corners1, ids1, rejectedImgPoints1 = aruco.detectMarkers(gray1, aruco_dict, parameters=parameters)
        corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(gray2, aruco_dict, parameters=parameters)        


        gray1, aruco_points1 =  sortArucoById(ids1, corners1, gray1)
        gray2, aruco_points2 =  sortArucoById(ids2, corners2, gray2)
        # print(aruco_points1)
        # exit()

        
        retval1, rvec1, tvec1 = cv2.solvePnP(ground, aruco_points1, cm1, dc1)
        retval2, rvec2, tvec2 = cv2.solvePnP(ground, aruco_points2, cm2, dc2)        

        
        axis_pts1, jac1 = cv2.projectPoints(axis, rvec1, tvec1, cm1, dc1)
        axis_pts2, jac2 = cv2.projectPoints(axis, rvec2, tvec2, cm2, dc2)
        

        gray1 = draw(gray1, aruco_points1, axis_pts1)
        gray2 = draw(gray2, aruco_points2, axis_pts2)        
        
                
        P1 = makeProjMatrix(cm1, rvec1, tvec1)
        P2 = makeProjMatrix(cm2, rvec2, tvec2)   


        target_point1 = targetArucoId(ids1, corners1)        
        target_point2 = targetArucoId(ids2, corners2)

        
        

        target_point1_ud = cv2.undistortPoints(target_point1, cm1, dc1, P=cm1)
        target_point2_ud = cv2.undistortPoints(target_point2, cm2, dc2, P=cm2)
        # print(target_point1_ud)
        # print(target_point2_ud)

        # exit()

        points4D = cv2.triangulatePoints(P1, P2, target_point1, target_point2)

        points3D = cv2.convertPointsFromHomogeneous(points4D.T)
        np.set_printoptions(suppress=True)
        np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
        print(points3D)
        print("")
        print("")
        exit()




        gray1 = aruco.drawDetectedMarkers(gray1, corners1)
        gray2 = aruco.drawDetectedMarkers(gray2, corners2)
             
        # Display the resulting frame
        cv2.imshow('image1',gray1)
        cv2.imshow('image2',gray2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except Exception as e:
        pass
        # raise e
 
# When everything done, release the capture
cap1.release()
cap2.release()
cv2.destroyAllWindows()