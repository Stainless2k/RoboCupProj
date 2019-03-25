import numpy as np
import cv2
import pickle

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# TODO: points like (x*square.len, y*square.len, 0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cam = cv2.VideoCapture('../cal4.3gp')
while(cam.isOpened()):

    ret, img = cam.read()
    if ret == True:
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
            if cv2.waitKey(500) & 0xFF == ord('q'):
                cam.release()
            if len(objpoints) > 50:
                cam.release()

print("out")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(mtx)
p = open('cal.pckl', 'wb')
pickle.dump((mtx, dist), p)
p.close()

cv2.destroyAllWindows()
