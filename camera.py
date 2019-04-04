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

	squarelen = 0.003
	chessheigth = 9
	chesslength = 6
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
	objp = np.zeros((chesslength * chessheigth,3), np.float32)
	grid = np.mgrid[0:9 * squarelen:squarelen,0:chesslength * squarelen:squarelen].T.reshape(-1,2)
	#no idea why mgrid does a (63,2) matrix here
	objp[:,:2] = grid[:chesslength * chessheigth,:]
	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.

	# Connect with ALMotion
	#motionProxy = ALProxy("ALMotion",roboIP, port)

	# wake up
	#motionProxy.wakeUp()

	# Connect with ALVideoDevice
	print()
	visionProxy = ALProxy("ALVideoDevice",roboIP,port)


	# Set up camera
	resolution = vision_definitions.kVGA
	colorSpace = 11
	fps = 1

	# Subscribe camera
	nameId = visionProxy.subscribeCamera("python_GVM",1,resolution, colorSpace, fps)


	# Get image
	#img = visionProxy.getImageRemote(nameId)

	# convert image
	#img = np.fromstring(img[6], dtype=np.uint8).reshape((480, 640, 3))

	if testing:
		#show image
		#cv2.imshow('img', img)
		#cv2.waitKey(10000)
		pass
	else:

		while len(objpoints) < 100:
			print("next Picture")
			img = visionProxy.getImageRemote(nameId)

			img = np.fromstring(img[6], dtype=np.uint8).reshape((480, 640, 3))

			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			ret, corners = cv2.findChessboardCorners(gray, (chessheigth ,chesslength), None)
			if ret == True:
				objpoints.append(objp)
				print(len(objpoints))
				corners2=cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
				imgpoints.append(corners)

				# Draw and display the corners
				cv2.drawChessboardCorners(img, (chessheigth ,chesslength), corners2, ret)
				cv2.imshow('img', img)
			else:
				cv2.imshow('img', img)

			# elif ret == False:
			# cam.release()
			#advance frames with anykey, close with q
			if cv2.waitKey(10) & 0xFF == ord('q'):
				# cam.release()
				break
		cv2.destroyAllWindows()
		print("out")
		ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
		print(mtx)
		p = open('test.pckl', 'wb')
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
