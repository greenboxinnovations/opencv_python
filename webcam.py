import numpy as np
import cv2


# ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
# ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

# ip_left =  "rtsp://192.168.0.140:8554/live0.264"
# ip_right =  "rtsp://192.168.0.141:8554/live0.264"

ip_left =  "rtsp://192.168.0.124:554/Streaming/Channels/1/?transportmode=unicast"
# ip_right = "rtsp://192.168.0.124:554/Streaming/Channels/1/?transportmode=unicast"

cap = cv2.VideoCapture(ip_left)

cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image1', 1280, 720)


while(True):
    # Capture frame-by-frame
    try:
    	ret, frame = cap.read()
    	# Our operations on the frame come here
    	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    	# Display the resulting frame
    	cv2.imshow('image1',gray)
    	if cv2.waitKey(1) & 0xFF == ord('q'):
    		break
    except Exception as e:
    	raise e
	    

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()