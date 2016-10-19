# NOTE: This is just an example of how we could layout the robot class
# I don't know if any of this will actually compile and run

import brickpi
import time

interface = brickpi.Interface();
robot = None;

class Robot:
	# attributes - ideally different components (motors, ultrasonic sensor, etc)
	motors = [0,1]
	
	

	# constructor
	def __init__(self):
		global interface
		interface = brickpi.Interface()
		interface.initialize()
		interface.motorEnable(self.motors[0])
		interface.motorEnable(self.motors[1])

		motorParams = interface.MotorAngleControllerParameters()
		motorParams.maxRotationAcceleration = 6.0
		motorParams.maxRotationSpeed = 12.0
		motorParams.feedForwardGain = 255/20.0
		motorParams.minPWM = 18.0
		motorParams.pidParameters.minOutput = -255
		motorParams.pidParameters.maxOutput = 255
		motorParams.pidParameters.k_p = 517
		motorParams.pidParameters.k_i = 1000
		motorParams.pidParameters.k_d = 13

		interface.setMotorAngleControllerParameters(self.motors[0],motorParams)
		interface.setMotorAngleControllerParameters(self.motors[1],motorParams)	


	def moveForwards(self, distance):
		# implementation
		angle = self.distToAngle(distance)

		interface.increaseMotorAngleReferences(self.motors,[angle,angle])

		while not interface.motorAngleReferencesReached(self.motors) :
			motorAngles = interface.getMotorAngles(self.motors)
			if motorAngles :
				print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
			#time.sleep(0.1)
	
	def distToAngle(self, dist):
		angle = float(dist * 15.0)/42.0 
		return angle

	def moveBackwards(self, distance):
		# implementation
			angle = -self.distToAngle(distance)

			interface.increaseMotorAngleReferences(self.motors,[angle,angle])

			while not interface.motorAngleReferencesReached(self.motors) :
				motorAngles = interface.getMotorAngles(self.motors)
				if motorAngles :
					print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
				#time.sleep(0.1)

	def rotateRight(self, rotAngle):
		# implementation
			angle = self.rotAngleToMotorAngle(rotAngle)
			interface.increaseMotorAngleReferences(self.motors,[angle,-angle])

			while not interface.motorAngleReferencesReached(self.motors) :
				motorAngles = interface.getMotorAngles(self.motors)
				if motorAngles :
					print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
				#time.sleep(0.1)
		

	def rotateLeft(self, angle):
		# implementation
		self.rotateRight(-angle)
	
	def rotAngleToMotorAngle(self, rotationAngle):
		return float(rotationAngle * 4.6) / 90.0
		
	# other member functions here...
	def moveSquare(self,distance):
		for i in range(0,4):
			self.moveForwards(distance)
			self.rotateLeft(90)
			time.sleep(0.05)
		
	#def check_drift():
		
# End of Robot Class

# main

robot = Robot()
for i in range(0, 10):
	robot.moveSquare(40)
	time.sleep(10)
interface.terminate()
