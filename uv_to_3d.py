import cv2
import cv2.aruco as aruco

import numpy as np

import collections
 
ip =  "rtsp://192.168.0.128:554/Streaming/Channels/1/?transportmode=unicast"
cap = cv2.VideoCapture(ip)

disp = True


fs = cv2.FileStorage("cam_128.yml", cv2.FILE_STORAGE_READ)
cm = fs.getNode("camera_matrix").mat()
dc = fs.getNode("distortion_coefficients").mat()

target_aruco_id = 11


# 0,0               60,0
# 
# 
# 
# 0,60              60,60

ground = np.float32([
			[0, 0, 0],
			[0.6, 0, 0],
			[0.6, 0.6, 0], 
			[0, 0.6, 0]]
			).reshape(-1, 3)
# print(ground) 


axis = np.float32([[0.2,0,0], [0,0.2,0], [0,0,-0.2]]).reshape(-1,3)

cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image', 1280, 720)



def draw(img, corners, imgpts):    
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


		if id_single != target_aruco_id:            
			# for plotting
			c_list.append(corners[i][0][0].tolist())            

			# for rearranging
			myDict[id_list[i][0]] = corners[i][0][0].tolist()         

	# plot first corner
	for x, p in enumerate(c_list):
		# print(p)
		# print(tuple(int(s) for s in p))            
		cv2.circle(img,tuple(int(s) for s in p), 10, (0,0,255), -1)


	image_points = np.zeros(shape=(len(myDict),2))
	# print(image_points1)

	od = collections.OrderedDict(sorted(myDict.items()))
	# print(od)
	counter=0
	for k, v in od.items(): 
		# print(k, v)
		# np.insert(image_points1,v[0],v[1],axis=0)
		image_points[counter] = [v[0],v[1]]	        
		counter=counter+1

	return img, image_points



def targetArucoId(ids, corners):

	# image_points = np.zeros(shape=(1,2))    

	for i, id_single in enumerate(ids):
		if id_single == target_aruco_id:            
			image_points = corners[i]

	return image_points	




while(True):

	try:
		# Capture frame-by-frame
		ret, frame = cap.read()        
				
		gray = frame
		# aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
		aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
		parameters =  aruco.DetectorParameters_create()
	 

		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)        


		gray, aruco_points =  sortArucoById(ids, corners, gray)        
		# print(aruco_points)
		# exit()

		
		retval, rvec, tvec = cv2.solvePnP(ground, aruco_points, cm, dc)        

		
		axis_pts, jac1 = cv2.projectPoints(axis, rvec, tvec, cm, dc)        
		

		gray = draw(gray, aruco_points, axis_pts)        
		
				
		P1 = makeProjMatrix(cm, rvec, tvec)


		gray = aruco.drawDetectedMarkers(gray, corners)        
			 
		# Display the resulting frame
		cv2.imshow('image',gray)      


		if disp:
			disp = not disp

			# get all aruco points
			target_point = targetArucoId(ids, corners)
			# print(target_point)
			# print(target_point.shape)

			# numpy friendly
			append = target_point[0]

			num_rows, num_cols = append.shape
			# print(num_rows, num_cols)

			ones_mat = np.ones(shape=(num_rows, 1))
			# print(rotation_mat)

			
			# make homogenous by adding 1 as z value
			z = np.append(append, ones_mat, axis=1)
			print(z[0])





			rotation_mat = np.zeros(shape=(3, 3))
			R = cv2.Rodrigues(rvec, rotation_mat)[0]
			# print(R)

			# print(np.linalg.inv(R))


			leftSideMat = np.zeros(shape=(3, 3))
			rightSideMat = np.zeros(shape=(3, 3))

			leftSideMat = np.linalg.inv(R) * np.linalg.inv(cm) * z[0]
			print(leftSideMat)

			rightSideMat = np.linalg.inv(R) * tvec
			print(rightSideMat)

			print(rightSideMat[2][0])
			print(leftSideMat[2][0])


			s = (1 + rightSideMat[2][0]/leftSideMat[2][0])
			print(s)

			# np.column_stack([uvPoint, v])
			# print(uvPoint)
			# print(uvPoint.shape)

			# cv::Mat uvPoint = (cv::Mat_<double>(3,1) << 363, 222, 1); // u = 363, v = 222, got this point using mouse callback

			# cv::Mat leftSideMat  = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
			# cv::Mat rightSideMat = rotationMatrix.inv() * tvec;

			# double s = (285 + rightSideMat.at<double>(2,0))/leftSideMat.at<double>(2,0); 

			# std::cout << "P = " << rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec) << std::endl;



			# rotation_mat = np.zeros(shape=(3, 3))
			# R = cv2.Rodrigues(rvec, rotation_mat)[0]

			# print(rotation_mat)
			# print(R)

			# if R.all() == rotation_mat.all():
			# 	print("yes")


		k = cv2.waitKey(33)
		if k == ord('q'):
			break

		if k == ord('d'):
			disp = not disp

		# if cv2.waitKey(1) & 0xFF == ord('q'):
		#     break
	except Exception as e:
		# pass
		raise e
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()