import almath
import math
import time
from naoqi import ALProxy

def main(roboIP, PORT=9559):
	motionProxy = ALProxy("ALMotion", roboIP, PORT)
	
	# WakeUP
	motionProxy.wakeUp()
	
	motionProxy.setStiffnesses("Head",1.0)
	
	# go to inithead pose
	names = ["HeadYaw","HeadPitch"]
	angels = [0.0, 0.0]
	times = [1.0,1.0]
	isAbsolute = True
	motionProxy.angleInterpolation(names,angels,times,isAbsolute)
	
	# look left
	names = "HeadYaw"
	angels = 90*almath.TO_RAD
	times = 4.0
	isAbsolute = True
	motionProxy.post.angleInterpolation(names,angels,times,isAbsolute)
	
	time.sleep(1.0)
	
	# look right
	# postponed until left turn is finished
	names = "HeadYaw"
	angels = -90*almath.TO_RAD
	times = 4.0
	isAbsolute = True
	motionProxy.post.angleInterpolation(names,angels,times,isAbsolute)
	
	# wait until done before switching off
	motionProxy.waitUntilMoveIsFinished()
	time.sleep(15.0)
	
	# end of program
	motionProxy.rest()
	
if __name__ == "__main__":
	main("10.0.7.14", 9559)