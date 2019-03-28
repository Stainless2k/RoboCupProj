import numpy as np
import cv2
import pickle
import vision_definitions

from naoqi import ALProxy

roboIp = "10.0.7.14"
PORT = 9559

motionProxy = ALProxy("ALMotion", roboIp, PORT)
	
	# wake up
motionProxy.wakeUp()

# import glob
# visionProxy = ALProxy("ALVideoDevice",roboIp,PORT)
visionProxy = ALProxy('RobocupVision', "10.0.7.14", 9559)

resolution = vision_definitions.kVGA
colorSpace = vision_definitions.kYUVColorSpace
fps = 2

nameId = visionProxy.subscribe("python_GVM",resolution, colorSpace, fps)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# 2,5cm squarelen in m
squarelen = 0.0025
chessheigth = 9
chesslength = 6
nFrame = 5

objp = np.zeros((chesslength * chessheigth,3), np.float32)
grid = np.mgrid[0:9 * squarelen:squarelen,0:chesslength * squarelen:squarelen].T.reshape(-1,2)
#no idea why mgrid does a (63,2) matrix here
objp[:,:2] = grid[:chesslength * chessheigth,:]
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# #video based cal
# cam = cv2.VideoCapture('../cal5.3gp')
# frame = 60
while(True):
    # frame = frame + 1
    # ret, img = cam.read()
    #only read every nth frame, there is probably a better way than this
    # if ret == True and frame > nFrame: 
    # frame = 0
	# img = visionProxy.getImageRemote(nameId)
	data = visionProxy.getBGR24Image(0)
	img = np.fromstring(data, dtype=np.uint8).reshape((480, 640, 3))
	gray = cv2.cvtColor(cv2.UMat(img), cv2.COLOR_BGR2GRAY)
    # find the chess board corners
	ret, corners = cv2.findChessboardCorners(gray, (chessheigth ,chesslength), None)
    # if found, add object points, image points (after refining them)
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
	if cv2.waitKey(100) & 0xFF == ord('q'):
		# cam.release()
		break

'''# img based cal
images = glob.glob('*.jpg')
for name in images:
    print(name)
    img = cv2.imread(name)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

    # If found, add object points, image points (after refining them)

    if ret == True:
        objpoints.append(objp)
        print(len(objpoints))
        corners2=cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        #cv2.drawChessboardCorners(img, (9,6), corners2, ret)
        #cv2.imshow('img', img)
        #cv2.waitKey(500)
'''
cv2.destroyAllWindows()
print("out")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(mtx)
p = open('calRobo.pckl', 'wb')
pickle.dump((mtx, dist), p)
p.close()

motionProxy.rest()

# cv2.destroyAllWindows()
