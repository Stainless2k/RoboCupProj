from naoqi import ALProxy

import numpy as np
import cv2
import vision_definitions
import pickle





def main():
	testing = False

	# Nao connection data
	roboIP = '10.0.7.14'
	port = 9559

	motionProxy = ALProxy("ALMotion",roboIP,port)
	names = "HeadPitch"
	angels = np.deg2rad(-10)
	times = 1.0
	isAbsolute = True
	motionProxy.setStiffnesses("Head", 1.0)
	motionProxy.angleInterpolation(names,angels,times,isAbsolute)
	motionProxy.setStiffnesses("Head", 0.0)

	#length of the chessboard squares in meters
	squarelen = 0.03
	chessheigth = 9
	chesslength = 6
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
	#arranging an array with the positions of the chessboard square cornerSubPix
	#[(0, 0, 0), (0.03, 0, 0), (0.06, 0, 0) ...]
	objp = np.zeros((chesslength * chessheigth,3), np.float32)
	grid = np.mgrid[0:9 * squarelen:squarelen,0:chesslength * squarelen:squarelen].T.reshape(-1,2)
	#no idea why mgrid does a (63,2) matrix here
	objp[:,:2] = grid[:chesslength * chessheigth,:]
	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.
	# Connect with ALVideoDevice
	visionProxy = ALProxy("ALVideoDevice",roboIP,port)
	# Set up camera
	resolution = vision_definitions.kVGA # 640 x 480
	colorSpace = 11 #RGB
	fps = 1
	# Subscribe camera
	nameId = visionProxy.subscribeCamera("python_GVM",1,resolution, colorSpace, fps)

	if testing:
		pass
	else:
		#we need at least 40 frames where we found the chessboard
		while len(objpoints) < 40:
			print("next Picture")
			#get image from robot
			img = visionProxy.getImageRemote(nameId)
			#converting to opencv format
			img = np.fromstring(img[6], dtype=np.uint8).reshape((480, 640, 3))
			#grayscale image for easier detection
			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			#find cheassboard
			ret, corners = cv2.findChessboardCorners(gray, (chessheigth ,chesslength), None)
			#if the chessboard is found add data to arrays
			if ret == True:
				objpoints.append(objp)
				print(len(objpoints))
				#increase accuracy of detected corners
				corners2=cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
				imgpoints.append(corners2)

				# Draw and display the corners
				cv2.drawChessboardCorners(img, (chessheigth ,chesslength), corners2, ret)
				cv2.imshow('img', img)
			#if no chessboard is found still show the image
			else:
				cv2.imshow('img', img)
			#end early by holidng Q
			if cv2.waitKey(10) & 0xFF == ord('q'):
				break

		cv2.destroyAllWindows()
		print("out")
		#use date to calibrate camera
		ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
		#save values to file
		p = open('calRobo.pckl', 'wb')
		pickle.dump((mtx, dist), p)
		p.close()

	# release image
	visionProxy.releaseImage(nameId)

	# unsubscribe camera
	visionProxy.unsubscribe(nameId)

	# end
	#motionProxy.rest()




if __name__ == "__main__":
	main()
