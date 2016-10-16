# NOTE: This is just an example of how we could layout the robot class
# I don't know if any of this will actually compile and run

import brickpi
import time

interface = None;
robot = None;

class Robot:
	# attributes - ideally different components (motors, ultrasonic sensor, etc)
	motors = [0,1]
	
	# constructor
	def __init__(self):
		global interface
		interface = brickpi.Interface()
		interface.initialize()
		interface.motorEnable(motors[0])
		interface.motorEnable(motors[1])

		motorParams = interface.MotorAngleControllerParameters()
		motorParams.maxRotationAcceleration = 6.0
		motorParams.maxRotationSpeed = 12.0
		motorParams.feedForwardGain = 255/20.0
		motorParams.minPWM = 18.0
		motorParams.pidParameters.minOutput = -255
		motorParams.pidParameters.maxOutput = 255
		motorParams.pidParameters.k_p = 100.0
		motorParams.pidParameters.k_i = 0.0
		motorParams.pidParameters.k_d = 0.0

		interface.setMotorAngleControllerParameters(motors[0],motorParams)
		interface.setMotorAngleControllerParameters(motors[1],motorParams)

	def moveForward(self, distance):
		# implementation

	def moveBackwards(self, distance):
		# implementation

	def rotateRight(self, angle):
		# implementation

	def rotateLeft(self, angle):
		# implementation

	# other member functions here...

# End of Robot Class

# main

robot = Robot()

# ...

interface.terminate()