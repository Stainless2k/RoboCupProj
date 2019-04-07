from naoqi import ALProxy
import numpy as np
import cv2
import cv2.aruco as aruco
import pickle
import vision_definitions
import math
import time

#Load calibration for pose estimation later
p = open('calRobo.pckl', 'rb')
cameraMatrix, distCoeffs = pickle.load(p)
p.close()
#Generate marker or marker board
para = aruco.DetectorParameters_create()
dic = aruco.Dictionary_get(aruco.DICT_6X6_1000)
board = aruco.GridBoard_create(
	markersX=1,
	markersY=1,
	markerLength=0.09,
	markerSeparation=0.01,
	dictionary=dic)
#Connection settings
roboIP = "10.0.7.14"
port = 9559
#Video settings
resolution = vision_definitions.kVGA #640 x 480
colorSpace = 11 #RGB colorspace
fps = 1
#Modul Proxys
motionProxy = ALProxy("ALMotion",roboIP,port)
visionProxy = ALProxy("ALVideoDevice",roboIP,port)
postureProxy = ALProxy("ALRobotPosture",roboIP,port)
visionProxy.setParameter(0, 22, 2)
#Subscribe to camera
nameId = visionProxy.subscribeCamera("python_GVM",1,resolution, colorSpace, fps)
#Turn value
turnD = np.deg2rad(15)

def main():
	# wake up
	motionProxy.setStiffnesses("Head", 1.0)
	print("Standing up...")
	motionProxy.wakeUp()
	postureProxy.goToPosture("StandInit", 1.0)
	print("Start search...")
	search()
	stopGoing()
	# unsubscribe camera
	visionProxy.unsubscribe(nameId)
	# end
	motionProxy.setStiffnesses("Head", 0.0)
	motionProxy.rest()
	print("Going to sleep...")

def turnLeft():
	x = 0.0
	y = 0.0
	theta = turnD
	freq = 0.1
	motionProxy.moveTo(x,y,theta)

def turnRight():
	x = 0.0
	y = 0.0
	theta = -1 * turnD
	freq = 0.1
	motionProxy.moveTo(x,y,theta)

def startGoing():
	x = 0.15 # 15cm
	y = 0
	theta = 0
	freq = 0.1
	motionProxy.moveTo(x,y,theta)

def lookDown():
	names = "HeadPitch"
	angels = np.deg2rad(20)
	times = 1.0
	isAbsolute = True
	motionProxy.angleInterpolation(names,angels,times,isAbsolute)

def lookUp():
	names = "HeadPitch"
	angels = np.deg2rad(-10)
	times = 1.0
	isAbsolute = True
	motionProxy.angleInterpolation(names,angels,times,isAbsolute)

def stopGoing():
	motionProxy.stopMove()

def look():
	print("Start loading the image...")
	#Get image from subcribedCamera
	img = visionProxy.getImageRemote(nameId)
	visionProxy.releaseImage(nameId)
	if img == None :
		print("Error: No Image received...")
		visionProxy.unsubscribe(nameId)
		motionProxy.rest()
		exit()
	#Converting to opencv format
	img = np.fromstring(img[6], dtype=np.uint8).reshape((480, 640, 3))
	print("Image loaded and converted...")

	#converting to grayscale for easier marker detection
	gImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	#draw crosshiar
	h, w, _ = img.shape
	img = cv2.line(img, (0, int(h/2)), (w, int(h/2)), (255, 0, 0))
	img = cv2.line(img, (int(w/2), 0), (int(w/2), h), (255, 0, 0))
	#detect markers
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gImg, dic, parameters = para)
	#find missing markers with help of already found markers, only usefull if you use a board with mulitple markers
	'''
	corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
		image = gImg,
		board = board,
		detectedCorners = corners,
		detectedIds = ids,
		rejectedCorners = rejectedImgPoints,
		cameraMatrix = cameraMatrix,
		distCoeffs = distCoeffs)
	'''
	#draw green border around detected markers (for debugging)
	img = aruco.drawDetectedMarkers(img, corners, borderColor=(0, 255, 0))
	#estimate position of the marker
	pose, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs)
	#if a positon is found
	if pose:
		#draw XYZ axis on marker (for debugging)
		img = aruco.drawAxis(img, cameraMatrix, distCoeffs, rvec, tvec, 0.3)
		#tvec[0] contains the postion of the marker on the x axis relative to camera middle point
		#if tvec[0] is close to 0 the marker must be ahead
		if abs(tvec[0]) < 0.05:
			ret = "ahead"
		#marker to the left
		elif tvec[0] < 0:
			ret = "left"
		#marker to the rigth
		elif tvec[0] > 0:
			ret = "right"
	#if no postion is detected we didnt find the marker
	else:
		ret = "notFound"
	#show the img (for debugging)
	cv2.imshow('img', img)
	return ret

def search():
	i = 0
	isLookDown = False
	lookUp()
	#Phase 1
	#should find the picture in 24 steps...24*15° = 360° ...
	while i < 25:
		i= i+1
		print("Searching the Marker: " , i)
		dir = look()
		print("Marker is " + dir)
		#if marker is ahead start phase 2
		if dir == "ahead":
			#marker is ahead
			startGoing()
			break
		#marker left
		elif dir == "left":
			turnLeft()
		#marker left
		elif dir == "right":
			turnRight()
		#if no marker is found, it migth be close, so we look at the ground
		elif dir == "notFound" and isLookDown == False:
			lookDown()
			isLookDown = True
		#if marker is not close we look up again and turn left
		elif dir == "notFound" and isLookDown == True:
			lookUp()
			turnLeft()
			isLookDown = False
		#this shouldnt happen
		else:
			print("Something odd just happened...")
		#waiting 3 seconds(to not overtax the robot) or until a key is pressed. Holding down Q exits Phase 1
		if cv2.waitKey(3000) & 0xFF == ord('q'):
			stopGoing()
			break
	#Phase 2
	print("Marker is ahead ....Going for it...")
	i = 0
	#15 * 15cm = 225m this should be enough to get to the marker
	while i < 15:
		i= i+1
		print("Trying to walk to the marker: " , i)
		dir = look()
		print("Marker is " + dir)
		#if we cant see the marker it must be close
		if dir == "notFound" and isLookDown == False:
			lookDown()
			isLookDown = True
		#if we cant find the marker after looking down we must be on it
		elif dir == "notFound" and isLookDown == True:
			print("Lost the Picture again. Am I standing on it?")
			break

		elif dir == "ahead":
			startGoing()
		#we migth have to readjust
		elif dir == "right":
			turnRight()
		#we migth have to readjust
		elif dir == "left":
			turnLeft()
		#this shouldnt happen
		else:
			print("Something odd just happened...")
		#waiting 3 seconds(to not overtax the robot) or until a key is pressed. Holding down Q exits Phase 2
		if cv2.waitKey(3000) & 0xFF == ord('q'):
			stopGoing()
			break

	print("Making sure im on it")
	#looking one more time to make sure we didnt stop early
	dir = look()
	#if the marker is found we start again
	if dir !=  "notFound":
		print("Im not standing on it")
		search()
	print("I am surely standing on the picture now...")

if __name__ == "__main__":
	main()
