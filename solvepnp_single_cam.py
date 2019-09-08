import cv2
import cv2.aruco as aruco

import numpy as np

import collections
 
ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

cap = cv2.VideoCapture(ip_left)

once = True
 
fs = cv2.FileStorage("intrinsics.yml", cv2.FILE_STORAGE_READ)
cm1 = fs.getNode("M1").mat()
dc1 = fs.getNode("D1").mat()

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


axis = np.float32([[0.3,0,0], [0,0.3,0], [0,0,-0.3]]).reshape(-1,3)


def draw(img, corners, imgpts):
    # corner = tuple(corners[0].ravel())
    corner = tuple(int(s) for s in corners[0].ravel())        
    img = cv2.line(img, corner, tuple(int(s) for s in imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(int(s) for s in imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(int(s) for s in imgpts[2].ravel()), (0,0,255), 5)
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
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()
     
        #print(parameters)
     
        '''    detectMarkers(...)
            detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
            mgPoints]]]]) -> corners, ids, rejectedImgPoints
            '''
            #lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # print(corners)



        # make dictionary to rearrange
        myDict = {}
        id_list = ids.tolist()

        # only first corner from aruco marker
        c_list = []
        for i, id_single in enumerate(ids):

            # for plotting
            c_list.append(corners[i][0][0].tolist())            

            # for rearranging
            myDict[id_list[i][0]] = corners[i][0][0].tolist()         

        # plot first corner
        for x, p in enumerate(c_list):
            # print(p)
            # print(tuple(int(s) for s in p))            
            cv2.circle(gray,tuple(int(s) for s in p), 10, (0,0,255), -1)


        # if once:
        #     once = False
            # rearrange by ids
            # print(myDict)

        image_points1 = np.zeros(shape=(4,2))
        # print(image_points1)

        od = collections.OrderedDict(sorted(myDict.items()))
        # print(od)
        for k, v in od.items(): 
            # print(k, v)
            # np.insert(image_points1,v[0],v[1],axis=0)
            image_points1[k-1] = [v[0],v[1]]

        # print(image_points1)


        retval, rvec, tvec = cv2.solvePnP(ground, image_points1, cm1, dc1)
        # print(tvec)
        # print(rvec)

        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvec, tvec, cm1, dc1)
        # print(imgpts)

        gray = draw(gray,image_points1,imgpts)

        # print(tuple(imgpts[0].ravel()))
        # exit()
        
        

        gray = aruco.drawDetectedMarkers(gray, corners)
             
        # Display the resulting frame
        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except Exception as e:
        pass
        # raise e
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()