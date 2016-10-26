import brickpi
import time
#from enum import Enum

interface = None

#class State(Enum):
	

class Robot:
	# attributes - ideally different components (motors, ultrasonic sensor, etc)
	motors = [0,1]
	
	touch_ports = [2,3]
	def __init__(self):
		global interface
		interface = brickpi.Interface()
		interface.initialize()
		interface.motorEnable(self.motors[0])
		interface.motorEnable(self.motors[1])
		interface.sensorEnable(self.touch_ports[0], brickpi.SensorType.SENSOR_TOUCH)
		interface.sensorEnable(self.touch_ports[1], brickpi.SensorType.SENSOR_TOUCH)
		motorParams = interface.MotorAngleControllerParameters()
		motorParams.maxRotationAcceleration = 6.0
		motorParams.maxRotationSpeed = 12.0
		motorParams.feedForwardGain = 255/20.0
		motorParams.minPWM = 18.0
		motorParams.pidParameters.minOutput = -255
		motorParams.pidParameters.maxOutput = 255
		motorParams.pidParameters.k_p = 800
		motorParams.pidParameters.k_i = 1000
		motorParams.pidParameters.k_d = 13

		interface.setMotorAngleControllerParameters(self.motors[0],motorParams)
		interface.setMotorAngleControllerParameters(self.motors[1],motorParams)	
		self.speed = 6.0

	# movement functions
	def setSpeed(self, newSpeed):
		 self.speed = newSpeed

	def reverseForkLeft(self,angle):
		interface.setMotorRotationSpeedReferences(self.motors, [0,0])
		self.moveBackwards(20)
		self.rotateLeft(angle)
		
			
	def reverseForkRight(self,angle):
                interface.setMotorRotationSpeedReferences(self.motors, [0,0])
		self.moveBackwards(20)
                self.rotateRight(angle)


	def moveForwards(self, distance=-1):
		if distance<0:

		
		
			while True:
				interface.setMotorRotationSpeedReferences(self.motors, [self.speed,self.speed])

				if self.checkSensors(self.touch_ports[0]) and not  self.checkSensors(self.touch_ports[1]):	
					self.reverseForkRight(45)
					interface.setMotorRotationSpeedReferences(self.motors, [self.speed,self.speed])
				

				elif not self.checkSensors(self.touch_ports[0]) and self.checkSensors(self.touch_ports[1]):
					self.reverseForkLeft(45)				
					interface.setMotorRotationSpeedReferences(self.motors, [self.speed,self.speed])


				elif self.checkSensors(self.touch_ports[0]) and  self.checkSensors(self.touch_ports[1]):
					self.reverseForkLeft(90)
					interface.setMotorRotationSpeedReferences(self.motors, [self.speed,self.speed])
				
		else:
			angle = self.distToAngle(distance)

			interface.increaseMotorAngleReferences(self.motors,[angle,angle])

			while not interface.motorAngleReferencesReached(self.motors) :
				if self.checkSensors(self.touch_ports[0]) or self.checkSensors(self.touch_ports[1]):
					break
				else:
					motorAngles = interface.getMotorAngles(self.motors)
					if motorAngles :
						print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]

	def moveBackwards(self, distance=-1):
		if distance<0:
			while True:
				interface.setMotorRotationSpeedReferences(self.motors, [-self.speed,-self.speed])
 				
  		else:
			angle = self.distToAngle(-distance)

			interface.increaseMotorAngleReferences(self.motors,[angle,angle])

			while not interface.motorAngleReferencesReached(self.motors):
				motorAngles = interface.getMotorAngles(self.motors)
				if motorAngles :
					print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]


	def rotateRight(self, rotAngle):
		angle = self.rotAngleToMotorAngle(rotAngle)
		interface.increaseMotorAngleReferences(self.motors,[angle,-angle])

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
		return float(rotationAngle * 4.7) / 90.0
		

	# other member functions
	def Left90deg(self):
		self.rotateLeft(90)
		
	def Right90deg(self):
		self.rotateRight(90)

	def moveSquare(self,distance):
		for i in range(0,4):
			self.moveForwards(distance)
			self.Left90deg()
			time.sleep(0.05)
	
	def checkSensors(self, touch_port):
		result=interface.getSensorValue(touch_port)
		if result:
			touched=result[0]
		else:
			touched=0

		return touched

	#def check_drift():
		
	# destructor
#	def __del__(self):
	#	interface.terminate()

# End of Robot Class

# main

robot = Robot()
robot.setSpeed(6)
robot.moveForwards()
interface.terminate()
