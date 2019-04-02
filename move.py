import math
from naoqi import ALProxy

		

def main(roboIp, PORT=9559):
	motionProxy = start(roboIp, PORT)
	while True:
		print ("1) move Forward")
		print ("2) move left")
		print ("3) move right")
		print ("4) stop")
		print ("0) Exit Program")
		x = input("Choose:")
		if x == 1: startGoing(motionProxy)
		if x == 2: turnLeft(motionProxy)
		if x == 3: turnRight(motionProxy)
		if x == 4: stopGoing(motionProxy)
		if x == 0: break
		
	stopGoing(motionProxy)
	stop(motionProxy)
	
		
def turnLeft(motionProxy):
	x = 0.0
	y = 0.0
	theta = math.pi/4.0
	freq = 0.1
	motionProxy.moveToward(x,y,theta,[["Frequency",freq]])
	
def turnRight(motionProxy):
	x = 0
	y = 0
	theta = -math.pi/4.0
	freq = 0.1
	motionProxy.moveToward(x,y,theta,[["Frequency",freq]])
	
def startGoing(motionProxy):
	x = 0.5
	y = 0
	theta = 0
	freq = 0.1
	motionProxy.moveToward(x,y,theta,[["Frequency",freq]])
	
def stopGoing(motionProxy):
	motionProxy.stopMove()
	
def start(roboIp, PORT=9559):
	motionProxy = ALProxy("ALMotion", roboIp, PORT)
	
	# wake up
	motionProxy.wakeUp()
	
	motionProxy.moveInit()
	
	return motionProxy
	
def stop(motionProxy):
	motionProxy.rest()
	
if __name__ == "__main__":
	main("10.0.7.14",9559)