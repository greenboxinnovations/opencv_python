import numpy as np
import cv2


ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

cap = cv2.VideoCapture(ip_right)



while(True):
    # Capture frame-by-frame
    try:
    	ret, frame = cap.read()
    	# Our operations on the frame come here
    	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    	# Display the resulting frame
    	cv2.imshow('frame',gray)
    	if cv2.waitKey(1) & 0xFF == ord('q'):
    		break
    except Exception as e:
    	raise e
	    

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()