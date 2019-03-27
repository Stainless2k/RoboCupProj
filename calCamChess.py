import numpy as np
import cv2
import pickle
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# 2,5cm squarelen in m
squarelen = 0.0025
chessheigth = 9
chesslength = 6
objp = np.zeros((chesslength * chessheigth,3), np.float32)
grid = np.mgrid[0:9 * squarelen:squarelen,0:chesslength * squarelen:squarelen].T.reshape(-1,2)
objp[:,:2] = grid[:chesslength * chessheigth,:]
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

#video based cal
cam = cv2.VideoCapture('../cal5.3gp')
frame = 60
while(cam.isOpened()):

    frame = frame + 1
    ret, img = cam.read()
    if ret == True and frame > 5:

        frame = 0
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
            cv2.drawChessboardCorners(img, (9,6), corners2, ret)
            cv2.imshow('img', img)


            #advance frames with anykey, close with q
        else:
            cv2.imshow('img', img)

    elif ret == False:
        cam.release()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cam.release()




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
p = open('calHandy.pckl', 'wb')
pickle.dump((mtx, dist), p)
p.close()

cv2.destroyAllWindows()
