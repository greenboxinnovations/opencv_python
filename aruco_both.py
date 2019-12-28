import cv2
import cv2.aruco as aruco
 
# ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
# ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

# ip_left =  "rtsp://192.168.0.123:554/Streaming/Channels/1/?transportmode=unicast"
# ip_right = "rtsp://192.168.0.124:554/Streaming/Channels/1/?transportmode=unicast"

ip_left =   "rtsp://192.168.0.140:8554/live0.264"
ip_right = "rtsp://192.168.0.141:8554/live0.264"

cap1 = cv2.VideoCapture(ip_left)
cap2 = cv2.VideoCapture(ip_right)

cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
cv2.namedWindow('image2', cv2.WINDOW_NORMAL)

cv2.resizeWindow('image1', 1280, 720)
cv2.resizeWindow('image2', 1280, 720)

 
while(True):
    # Capture frame-by-frame
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters =  aruco.DetectorParameters_create()
 
    #print(parameters)
 
    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners1, ids1, rejectedImgPoints1 = aruco.detectMarkers(frame1, aruco_dict, parameters=parameters)
    corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(frame2, aruco_dict, parameters=parameters)
    # print(corners1)    
 
    #It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs
 
    gray1 = aruco.drawDetectedMarkers(frame1, corners1, ids1)
    gray2 = aruco.drawDetectedMarkers(frame2, corners2, ids2)
 
    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('image1',gray1)
    cv2.imshow('image2',gray2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
# When everything done, release the capture
cap1.release()
cap2.release()
cv2.destroyAllWindows()