import cv2
import cv2.aruco as aruco

import numpy as np

# import collections
from collections import OrderedDict
 
# ip_left =  "rtsp://admin:admin123@192.168.0.129:554/Streaming/Channels/1/?transportmode=unicast"
# ip_right = "rtsp://admin:admin123@192.168.0.130:554/Streaming/Channels/1/?transportmode=unicast"

ip_left1 =  "rtsp://192.168.0.123:554/Streaming/Channels/1/?transportmode=unicast"
ip_right1 = "rtsp://192.168.0.124:554/Streaming/Channels/1/?transportmode=unicast"

ip_left2 =   "rtsp://192.168.0.140:8554/live0.264"
ip_right2 = "rtsp://192.168.0.141:8554/live0.264"

cap1 = cv2.VideoCapture(ip_left1)
cap2 = cv2.VideoCapture(ip_right1)

cap3 = cv2.VideoCapture(ip_left2)
cap4 = cv2.VideoCapture(ip_right2)

cap1.set(cv2.CAP_PROP_BUFFERSIZE, 9);
cap2.set(cv2.CAP_PROP_BUFFERSIZE, 9);
cap3.set(cv2.CAP_PROP_BUFFERSIZE, 9);
cap4.set(cv2.CAP_PROP_BUFFERSIZE, 9);

disp = True
 

fs1 = cv2.FileStorage("cam_123.yml", cv2.FILE_STORAGE_READ)
fs2 = cv2.FileStorage("cam_124.yml", cv2.FILE_STORAGE_READ)
fs3 = cv2.FileStorage("cam_140.yml", cv2.FILE_STORAGE_READ)
fs4 = cv2.FileStorage("cam_141.yml", cv2.FILE_STORAGE_READ)

cm1 = fs1.getNode("camera_matrix").mat()
dc1 = fs1.getNode("distortion_coefficients").mat()

cm2 = fs2.getNode("camera_matrix").mat()
dc2 = fs2.getNode("distortion_coefficients").mat()

cm3 = fs3.getNode("camera_matrix").mat()
dc3 = fs3.getNode("distortion_coefficients").mat()

cm4 = fs4.getNode("camera_matrix").mat()
dc4 = fs4.getNode("distortion_coefficients").mat()

fs1.release()
fs2.release()
fs3.release()
fs4.release()


# 0,0               30.5,0
# 
# 
# 
# 0,20              30.5,20
quad_1 = np.float32([
					[0, 	0, 		0],
					[0.295, 0, 		0],
					[0.295, 0.21, 	0], 
					[0, 	0.21, 	0]]
					).reshape(-1, 3)
# 0,21               30.5,21
# 
# 
# 
# 0,42              30.5,42
quad_2 = np.float32([
					[0, 	0.21, 0],
					[0.295, 0.21, 0],
					[0.295, 0.42, 0], 
					[0, 	0.42, 0]]
					).reshape(-1, 3)
# print(ground) 

solve = False
bool_aruco = False

axis = np.float32([[0.15,0,0], [0,0.15,0], [0,0,-0.15]]).reshape(-1,3)
axis2 = np.float32([[0.15,0.21,0], [0,0.36,0], [0,0.21,-0.15]]).reshape(-1,3)





cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image1', 1280, 720)

cv2.namedWindow('image2', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image2', 1280, 720)

cv2.namedWindow('image3', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image3', 1280, 720)

cv2.namedWindow('image4', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image4', 1280, 720)


# aruco init
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()


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


'''
sortArucoById: 		selects and sorts aruco ids either in ascending order, or by user supplied order

param ids: 			ids from aruco.detectMarkers()
param corners: 		corners from aruco.detectMarkers()
param img: 			img to draw aruco corners on
param id_order: 	select desired id's from all ids, eg. [1,2,3,10] or [10,3,1,2]
param ordered: 		if set to False [10,3,1,2] will not be ordered to [1,2,3,10] 

return: 
	img:			opencv mat with markers drawn 			
	image_points:	numpy array
'''
def sortArucoById(ids, corners, img, id_order, ordered=True):

	try:
		# make dictionary to rearrange
		myDict = {}
		id_list = ids.tolist()


		# id_order = [6,5,4,2,1]


		# only first corner from aruco marker
		corner_list = []
		for i, id_single in enumerate(ids):

			if id_single in id_order:

				corner_list.append(corners[i][0][0].tolist())
				myDict[id_list[i][0]] = corners[i][0][0].tolist()
			# if id_single != 11:            
			#     # for plotting
			#     c_list.append(corners[i][0][0].tolist())            

			#     # for rearranging
			#     myDict[id_list[i][0]] = corners[i][0][0].tolist()
		# print(myDict)

		# plot first corner
		for x, p in enumerate(corner_list):
			# print(p)
			# print(tuple(int(s) for s in p))            
			cv2.circle(img,tuple(int(s) for s in p), 10, (0,0,255), -1)

		# print(len(myDict))
		# image_points = np.zeros(shape=(4,2))
		image_points = np.zeros(shape=(len(myDict),2))
		# print(image_points)


		# myorder = [6,5,4,3,2,1]
		if ordered:
			od = OrderedDict(sorted(myDict.items()))
		else:
			od = OrderedDict((k, myDict[k]) for k in id_order)
				
		# od = collections.OrderedDict(sorted(myDict.items()))
		# od = OrderedDict(sorted(myDict.items()))
		# print(od)
		counter=0
		for k, v in od.items(): 
			print(k, v)
			# np.insert(image_points1,v[0],v[1],axis=0)
			# image_points[k-1] = [v[0],v[1]]	        
			image_points[counter] = [v[0],v[1]]	        
			counter=counter+1
		print("-")
		return img, image_points

	except Exception as e:
		raise e
		# print(e)
		# pass


	



def targetArucoId(ids, corners):

	# image_points = np.zeros(shape=(1,2))    

	for i, id_single in enumerate(ids):
		if id_single == 11:
			image_points = corners[i]

	return image_points



while(True):

	try:
		# Capture frame-by-frame
		ret1, gray1 = cap1.read()
		ret2, gray2 = cap2.read()

		ret3, gray3 = cap3.read()
		ret4, gray4 = cap4.read()


		if bool_aruco:
			if ret1:
				corners1, ids1, rejectedImgPoints1 = aruco.detectMarkers(gray1, aruco_dict, parameters=parameters)
			else:
				print("cap1")
				cap1 = cv2.VideoCapture(ip_left1)

			if ret2:
				corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(gray2, aruco_dict, parameters=parameters)
			else:
				print("cap2")
				cap2 = cv2.VideoCapture(ip_right1)

			if ret3:
				corners3, ids3, rejectedImgPoints3 = aruco.detectMarkers(gray3, aruco_dict, parameters=parameters)
			else:
				print("cap3")
				cap3 = cv2.VideoCapture(ip_left2)

			if ret4:
				corners4, ids4, rejectedImgPoints4 = aruco.detectMarkers(gray4, aruco_dict, parameters=parameters)
			else:
				print("cap4")
				cap4 = cv2.VideoCapture(ip_right2)

		# print("c1: "+ str(len(corners1)))
		# print("c2: "+ str(len(corners2)))
		# print("c3: "+ str(len(corners3)))
		# print("c4: "+ str(len(corners4)))        



		if solve:
			if((len(corners1) > 0) and (len(corners1) == len(corners2))):
				# solve = False
				gray1, aruco_points1 =  sortArucoById(ids1, corners1, gray1, [1,2,3,4])
				gray2, aruco_points2 =  sortArucoById(ids2, corners2, gray2, [1,2,3,4])

				gray3, aruco_points3 =  sortArucoById(ids3, corners3, gray3, [4,3,6,5], ordered=False)
				gray4, aruco_points4 =  sortArucoById(ids4, corners4, gray4, [4,3,6,5], ordered=False)
				# print(aruco_points3)
				# print(aruco_points4)
				# exit()


				retval1, rvec1, tvec1 = cv2.solvePnP(quad_1, aruco_points1, cm1, dc1)
				retval2, rvec2, tvec2 = cv2.solvePnP(quad_1, aruco_points2, cm2, dc2)

				retval3, rvec3, tvec3 = cv2.solvePnP(quad_2, aruco_points3, cm3, dc3)
				retval4, rvec4, tvec4 = cv2.solvePnP(quad_2, aruco_points4, cm4, dc4)


				axis_pts1, jac1 = cv2.projectPoints(axis, rvec1, tvec1, cm1, dc1)
				axis_pts2, jac2 = cv2.projectPoints(axis, rvec2, tvec2, cm2, dc2)

				axis_pts3, jac3 = cv2.projectPoints(axis2, rvec3, tvec3, cm3, dc3)
				axis_pts4, jac4 = cv2.projectPoints(axis2, rvec4, tvec4, cm4, dc4)


				gray1 = draw(gray1, aruco_points1, axis_pts1)
				gray2 = draw(gray2, aruco_points2, axis_pts2)

				gray3 = draw(gray3, aruco_points3, axis_pts3)
				gray4 = draw(gray4, aruco_points4, axis_pts4)

		        
		# P1 = makeProjMatrix(cm1, rvec1, tvec1)
		# P2 = makeProjMatrix(cm2, rvec2, tvec2)   


		# target_point1 = targetArucoId(ids1, corners1)        
		# target_point2 = targetArucoId(ids2, corners2)




		# target_point1_ud = cv2.undistortPoints(target_point1, cm1, dc1, P=cm1)
		# target_point2_ud = cv2.undistortPoints(target_point2, cm2, dc2, P=cm2)
		# # print(target_point1_ud)
		# # print(target_point2_ud)

		# # exit()
		# if disp:
		#     disp = False
		#     points4D = cv2.triangulatePoints(P1, P2, target_point1, target_point2)

		#     points3D = cv2.convertPointsFromHomogeneous(points4D.T)
		#     np.set_printoptions(suppress=True)
		#     np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
		#     print(points3D)
		#     print("")
		#     print("")
		# exit()

		if bool_aruco:
			try:
				gray1 = aruco.drawDetectedMarkers(gray1, corners1)
				gray2 = aruco.drawDetectedMarkers(gray2, corners2)
				gray3 = aruco.drawDetectedMarkers(gray3, corners3)
				gray4 = aruco.drawDetectedMarkers(gray4, corners4)
			except Exception as e:
				print(e)

		if ret1:			
			cv2.imshow('image1', gray1)
		if ret2:						
			cv2.imshow('image2', gray2)
		if ret3:			
			cv2.imshow('image3', gray3)
		if ret4:			
			cv2.imshow('image4', gray4)


		k = cv2.waitKey(300)
		if k == ord('q'):
			break

		elif k == ord('d'):
			disp = not disp

		elif k == ord('s'):
			print("Solve")
			# solve = True
			solve = not solve

		elif k == ord('a'):
			print("aruco")
			bool_aruco = not bool_aruco



	except Exception as e:
		# print(e)
		# pass
		raise e
 
# When everything done, release the capture
cap1.release()
cap2.release()
cv2.destroyAllWindows()