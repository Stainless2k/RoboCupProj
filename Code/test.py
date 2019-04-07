from naoqi import ALProxy
#tts = ALProxy("ALTextToSpeech","10.0.7.14", 9559)
#tts.say("Hello,world!")
import numpy as np
roboIP = "10.0.7.14"
port = 9559

motionProxy = ALProxy("ALMotion",roboIP,port)
motionProxy.wakeUp()
motionProxy.moveInit()
'''
x = 0.0
y = 0.0
theta = np.deg2rad(15)
freq = 0.1
motionProxy.moveTo(x,y,theta)
motionProxy.waitUntilMoveIsFinished()
motionProxy.rest()
'''
motionProxy.setStiffnesses("Head", 1.0)
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

lookUp()
lookDown()

lookUp()
lookDown()

lookUp()
lookDown()

lookUp()
lookDown()

lookUp()
lookDown()

motionProxy.setStiffnesses("Head", 0.0)
