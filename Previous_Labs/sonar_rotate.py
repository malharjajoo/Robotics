import brickpi
import time

interface = None

class Robot:
	# attributes - ideally different components (motors, ultrasonic sensor, etc)
	motors = [0,2]
	
	
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


	# movement functions
	def moveForwards(self, distance):
		angle = self.distToAngle(distance)

		interface.increaseMotorAngleReferences(self.motors,[angle,angle])

		while not interface.motorAngleReferencesReached(self.motors) :
			motorAngles = interface.getMotorAngles(self.motors)
			if motorAngles :
				print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]

	def moveBackwards(self, distance):
		self.moveForwards(-distance)

	def rotateRight(self, rotAngle):
		angle = self.rotAngleToMotorAngle(rotAngle)
		interface.increaseMotorAngleReferences(self.motors,[0,angle])

		while not interface.motorAngleReferencesReached(self.motors) :
			motorAngles = interface.getMotorAngles(self.motors)
			if motorAngles :
				print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		
	def rotateLeft(self, angle):
		self.rotateRight(-angle)


	# conversion functions
	def distToAngle(self, dist):
		#41.5 - w/ wheels
		angle = float(dist * 15.0)/41 
		return angle
	
	def rotAngleToMotorAngle(self, rotationAngle):
		#4.55 - w/ wheels
		return float(rotationAngle * 1.5) / 90.0
		

	# other member functions
	def Left90deg(self):
		self.rotateLeft(90)
		
	def Right90deg(self):
		self.rotateRight(90)


	#def check_drift():
		
	# destructor
#	def __del__(self):
	#	interface.terminate()

# End of Robot Class

# main

robot = Robot()
for i in range(45):
	
	robot.rotateRight(2)
#robot.moveForwards(40)
#robot.Left90deg()
#robot.Left90deg()
#robot.Left90deg()
#robot.Left90deg()
interface.terminate()

