from naoqi import ALProxy

motionProxy = ALProxy("ALMotion","10.0.7.14", 9559) 

motionProxy.wakeUp()

motionProxy.rest()