import cv2
import cv2.aruco as aruco

import collections
 
ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

cap = cv2.VideoCapture(ip_right)

once = True
 
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
        for p in c_list:
            # print(p)
            # print(tuple(int(s) for s in p))                
            cv2.circle(gray,tuple(int(s) for s in p), 10, (0,0,255), -1)


        if once:
            once = False
            # rearrange by ids
            # print(myDict)
            od = collections.OrderedDict(sorted(myDict.items()))
            # print(od)
            for k, v in od.items(): print(k, v)
            
     
             
        gray = aruco.drawDetectedMarkers(gray, corners)
             
        # Display the resulting frame
        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except Exception as e:
        raise e
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()