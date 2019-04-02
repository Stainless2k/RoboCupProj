from naoqi import ALProxy
import numpy as np
import cv2
import cv2.aruco as aruco
import pickle
import vision_definitions
import math
import time

#load cal
p = open('calRobo.pckl', 'rb')
cameraMatrix, distCoeffs = pickle.load(p)
p.close()
#Generate board
para = aruco.DetectorParameters_create()
dic = aruco.Dictionary_get(aruco.DICT_6X6_1000)
board = aruco.GridBoard_create(
	markersX=2,
	markersY=2,
	markerLength=0.09,
	markerSeparation=0.01,
	dictionary=dic)
#con settings
roboIP = "10.0.7.14"
port = 9559
#vid settings
resolution = vision_definitions.kVGA
colorSpace = 11
fps = 1
#proxys
motionProxy = ALProxy("ALMotion",roboIP,port)
visionProxy = ALProxy("ALVideoDevice",roboIP,port)
# subscribe camera
nameId = visionProxy.subscribe("python_GVM",resolution, colorSpace, fps)

turnD = 10

def main():
	# wake up
	motionProxy.setStiffnesses("Head", 1.0)
	motionProxy.wakeUp()
	#motionProxy.setMoveArmsEnabled(True, True)
	motionProxy.moveInit()
	search()
	stopGoing()
	# unsubscribe camera
	visionProxy.unsubscribe(nameId)
	# end
	motionProxy.setStiffnesses("Head", 0.0)
	motionProxy.rest()
	print("exit")

def turnLeft():
	x = 0.0
	y = 0.0
	theta = np.deg2rad(turnD)
	freq = 0.1
	#motionProxy.moveInit()
	motionProxy.moveTo(x,y,theta)

def turnRight():
	x = 0.0
	y = 0.0
	theta = np.deg2rad(-1 * turnD)
	freq = 0.1
	#motionProxy.moveInit()
	motionProxy.moveTo(x,y,theta)

def startGoing():
	x = 0.2
	y = 0
	theta = 0
	freq = 0.1
	#motionProxy.moveInit()
	motionProxy.moveTo(x,y,theta)


def lookDown():
	names = "HeadPitch"
	angels = np.deg2rad(20)
	times = 1.0
	isAbsolute = True
	motionProxy.angleInterpolation(names,angels,times,isAbsolute)

def lookUp():
	names = "HeadPitch"
	angels = 0.0
	times = 1.0
	isAbsolute = True
	motionProxy.angleInterpolation(names,angels,times,isAbsolute)

def stopGoing():
	motionProxy.stopMove()

def startModul(modul):
	return ALProxy(modul,roboIP,port)

def look():
	# get image
	#cv2.waitKey(2000)
	print("pic1")
	img = visionProxy.getImageRemote(nameId)
	# get the binary values of the image
	img = np.fromstring(img[6], dtype=np.uint8).reshape((480, 640, 3))
	print("pi2")
	# detect chessboard --------------------------------
	h, w, _ = img.shape
	#remove colors
	gImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	#draw crosshiar
	img = cv2.line(img, (0, int(h/2)), (w, int(h/2)), (255, 0, 0))
	img = cv2.line(img, (int(w/2), 0), (int(w/2), h), (255, 0, 0))
	#detect markers
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gImg, dic, parameters = para)
	#find missing markers with help of already found markers
	corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
		image = gImg,
		board = board,
		detectedCorners = corners,
		detectedIds = ids,
		rejectedCorners = rejectedImgPoints,
		cameraMatrix = cameraMatrix,
		distCoeffs = distCoeffs)
	#draw markers
	img = aruco.drawDetectedMarkers(img, corners, borderColor=(0, 255, 0))
	pose, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs)

	if pose:
		img = aruco.drawAxis(img, cameraMatrix, distCoeffs, rvec, tvec, 0.3)
		print(tvec)
		if abs(tvec[0]) < 0.05:
			ret = "ahead"
		elif tvec[0] < 0:
			ret = "left"
		elif tvec[0] > 0:
			ret = "right"
	else:
		ret = "notFound"

	#img = cv2.imsize(img, (640,480))
	cv2.imshow('img', img)
	return ret 

def search():

	i = 0
	isLookDown = False
	#find marker and positon robo staright
	while i < 100:
		i= i+1
		print(i)
		dir = look()
		print(dir)
		if dir == "ahead":
			#marker is ahead
			break

		elif dir == "left":
			turnLeft()

		elif dir == "right":
			turnRight()

		elif dir == "notFound" and isLookDown == False:
			lookDown()
			isLookDown = True

		elif dir == "notFound" and isLookDown == True:
			lookUp()
			turnLeft()
			isLookDown = False

		else:
			print("HOW")

		if cv2.waitKey(3000) & 0xFF == ord('q'):
			stopGoing()
			break

	print("phase 2")
	i = 0
	while i < 10:
		i= i+1
		print(i)
		dir = look()
		if dir == "notFound" and isLookDown == False:
			lookDown()
			isLookDown = True
		elif dir == "notFound" and isLookDown == True:
			print("maker lost")
			startGoing()
			break
		else:
			startGoing()

		if cv2.waitKey(3000) & 0xFF == ord('q'):
			stopGoing()
			break


'''
		if notFound:
			if pose:
				print("found")
			elif markerAhead:
				names = "HeadPitch"
				angels = 20.0
				times = 4.0
				isAbsolute = True
				motionProxy.post.angleInterpolation(names,angels,times,isAbsolute)
				cv2.imshow('img', img)
			else:
				print("not found")
				turnLeft(motionProxy)

		if pose:
			notFound = False
			# move axis to middle
			#tvec[0] = tvec[0] - 0.1
			#tvec[1] = tvec[1] - 0.1

		else:
			cv2.imshow('img', img)

	#advance frames with anykey, close with q
	if cv2.waitKey(1) & 0xFF == ord('q'):
		stopGoing(motionProxy)
		break
'''



if __name__ == "__main__":
	main()