import cv2
import cv2.aruco as aruco
import pickle
import numpy as np

#load cal
p = open('calHandy.pckl', 'rb')
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

'''
#write file
jpg = board.draw((1000, 1000))
cv2.imwrite("board.jpg", jpg)

#display
cv2.imshow('board', jpg)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''

cam = cv2.VideoCapture('../test14.3gp')

while(cam.isOpened()):

    ret, img = cam.read()
    if ret == True:

        h, w, _ = img.shape
        #remove colors
        gImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

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
        if pose :
            # move axis to middle
            #tvec[0] = tvec[0] - 0.1
            #tvec[1] = tvec[1] - 0.1
            if abs(tvec[0]) < 0.05:
                print("stop")
            elif tvec[0] < 0:
                print("left")
            elif tvec[0] > 0:
                print("right")
            img = aruco.drawAxis(img, cameraMatrix, distCoeffs, rvec, tvec, 0.3)
            print(tvec)
        cv2.imshow('img', img)

        #advance frames with anykey, close with q
        if cv2.waitKey(0) & 0xFF == ord('q'):
                cam.release()
    else:
        cam.release()


cv2.destroyAllWindows()
