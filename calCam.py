import cv2
import cv2.aruco as aruco
import pickle

para = aruco.DetectorParameters_create()
dic = aruco.Dictionary_get(aruco.DICT_5X5_1000)
board = aruco.GridBoard_create(
    markersX=4,
    markersY=5,
    markerLength=0.04,
    markerSeparation=0.01,
    dictionary=dic)

#write file
jpg = board.draw((1000, 1000))
cv2.imwrite("cal_board.jpg", jpg)

#display
cv2.imshow('board', jpg)
cv2.waitKey(0)
cv2.destroyAllWindows()

cam = cv2.VideoCapture('cal6.3gp')
print("go")
while(cam.isOpened()):
    ret, img = cam.read()
    if ret == True:
        #remove colors
        gImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #find markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gImg, dic, parameters=para)
        img = aruco.drawDetectedMarkers(img, corners, borderColor=(0, 255, 0))
        cv2.imshow('img', img)
        if cv2.waitKey(0) & 0xFF == ord('q'):
                break
        #markers were found
        if ids is not None and corners is not None and len(ids) > 0 and len(corners) > 0 and len(corners) == len(ids):
            #found all markers
            if len(ids) == len(board.ids):
                #black magic calibration method
                ret, cameraMatrix, distCoeffs, _, _ = cv2.calibrateCamera(
                        objectPoints=board.objPoints,
                        imagePoints=corners,
                        imageSize=gImg.shape,
                        cameraMatrix=None,
                        distCoeffs=None)

                #print cal valuse
                print(cameraMatrix)
                print(distCoeffs)
                #save values
                p = open('cal.pckl', 'wb')
                pickle.dump((cameraMatrix, distCoeffs), p)
                p.close()

                break

    #Q to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
