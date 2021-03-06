import brickpi
import time
#from enum import Enum
import CircularBuffer

interface = None

#class State(Enum):
	

class Robot:
	# attributes - ideally different components (motors, ultrasonic sensor, etc)
	motors = [0,1]
	speed = 6.0
	touch_ports = [0,1]
	sonar_port = 3
	usSensorBuffer = CircularBuffer.CircularBuffer()
	def __init__(self):
		global interface
		interface = brickpi.Interface()
		interface.initialize()
		interface.motorEnable(self.motors[0])
		interface.motorEnable(self.motors[1])
		interface.sensorEnable(self.touch_ports[0], brickpi.SensorType.SENSOR_TOUCH)
		interface.sensorEnable(self.touch_ports[1], brickpi.SensorType.SENSOR_TOUCH)
		interface.sensorEnable(self.sonar_port, brickpi.SensorType.SENSOR_ULTRASONIC)
		motorParams = interface.MotorAngleControllerParameters()
		motorParams.maxRotationAcceleration = 6.0
		motorParams.maxRotationSpeed = 12.0
		motorParams.feedForwardGain = 255/20.0
		motorParams.minPWM = 3.0
		motorParams.pidParameters.minOutput = -255
		motorParams.pidParameters.maxOutput = 255
		#position ctrl: 517, 1000, 13
		#velocity ctrl: 100, 0, 0
		motorParams.pidParameters.k_p = 517
		motorParams.pidParameters.k_i = 1000
		motorParams.pidParameters.k_d = 13

		interface.setMotorAngleControllerParameters(self.motors[0],motorParams)
		interface.setMotorAngleControllerParameters(self.motors[1],motorParams)	

	# movement functions
	def setSpeed(self, newSpeed):
		 self.speed = newSpeed

	def reverseForkLeft(self,angle):
		print("reversing back now...")
		interface.setMotorPwm(self.motors[0],0)
		interface.setMotorPwm(self.motors[1],0)
		print("moving back now....")
		self.moveBackwards(20)
		
		self.rotateLeft(angle)
		
			
	def reverseForkRight(self,angle):
		interface.setMotorPwm(self.motors[0],0)
		interface.setMotorPwm(self.motors[1],0)
		self.moveBackwards(20)
                self.rotateRight(angle)


	def moveForwards(self, distance=-1):
		if distance<0:
			self.setMotorRotationSpeed(self.speed, self.speed)
			while True:

				if self.checkSensors(self.touch_ports[0]) and not  self.checkSensors(self.touch_ports[1]):	
					self.reverseForkRight(90)
					self.setMotorRotationSpeed(self.speed, self.speed)
				

				elif not self.checkSensors(self.touch_ports[0]) and self.checkSensors(self.touch_ports[1]):
					self.reverseForkLeft(90)				
					self.setMotorRotationSpeed(self.speed, self.speed)


				elif self.checkSensors(self.touch_ports[0]) and  self.checkSensors(self.touch_ports[1]):
					self.reverseForkLeft(90)
					self.setMotorRotationSpeed(self.speed, self.speed)
				
		else:
			angle = self.distToAngle(distance)
			self.increaseMotorAngle(angle, angle)

	def moveBackwards(self, distance=-1):
		if distance<0:
			self.setMotorRotationSpeed(-self.speed, -self.speed)
 			while True:
				# sensor checks here
				time.sleep(1)	
  		else:
			angle = self.distToAngle(-distance)
			self.increaseMotorAngle(angle, angle)

	def rotateRight(self, rotAngle):
		angle = self.rotAngleToMotorAngle(rotAngle)
		self.increaseMotorAngle(angle, -angle)

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
	
	#used as wrapper for setting different pid values
	def setMotorRotationSpeed(self, speed1, speed2):
		motorParams = interface.MotorAngleControllerParameters()
		motorParams.maxRotationAcceleration = 4.0
		motorParams.maxRotationSpeed = 12.0
		motorParams.feedForwardGain = 255/20.0
		motorParams.minPWM = 1.0
		motorParams.pidParameters.minOutput = -255
		motorParams.pidParameters.maxOutput = 255
		motorParams.pidParameters.k_p = 100
		motorParams.pidParameters.k_i = 0
		motorParams.pidParameters.k_d = 0
		interface.setMotorAngleControllerParameters(self.motors[0],motorParams)
		interface.setMotorAngleControllerParameters(self.motors[1],motorParams)
		
		interface.setMotorRotationSpeedReferences(self.motors, [speed1,speed2])	

	#wraper for motor rotation
	def increaseMotorAngle(self, angle1, angle2):
		motorParams = interface.MotorAngleControllerParameters()
		motorParams.maxRotationAcceleration = 6.0
		motorParams.maxRotationSpeed = 12.0
		motorParams.feedForwardGain = 255/20.0
		motorParams.minPWM = 3.0
		motorParams.pidParameters.minOutput = -255
		motorParams.pidParameters.maxOutput = 255
		#517 , 1000,13		
		motorParams.pidParameters.k_p = 517
		motorParams.pidParameters.k_i = 1000
		motorParams.pidParameters.k_d = 13
		interface.setMotorAngleControllerParameters(self.motors[0],motorParams)
		interface.setMotorAngleControllerParameters(self.motors[1],motorParams)

		interface.increaseMotorAngleReferences(self.motors,[angle1,angle2])

		while not interface.motorAngleReferencesReached(self.motors) :
			if self.checkSensors(self.touch_ports[0]) or self.checkSensors(self.touch_ports[1]):
				break
			else:
				motorAngles = interface.getMotorAngles(self.motors)
				if motorAngles :
					print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]


	def readUsSensor(self, circularBuffer):
		#first fill up the circular buffer if it isn't already
		usReading = interface.getSensorValue(self.sonar_port)
		if usReading:
			print(usReading)
		else:
			print "Failed US Reading"
		circularBuffer.add(usReading[0])
		print circularBuffer.circularBuffer
		return circularBuffer.getMedian()

	def MoveForwardsWithSonar(self, safeDistance):
		while True:
			usReading = self.readUsSensor(self.usSensorBuffer)	#returns the median of the circular buffer
			print "median=",usReading
			error = usReading - safeDistance
			k = float(error)/30.0	#k gain - adjust for different smoothness
			if k > 1:
				k = 1
			if k < -1:
				k = -1
			print "k=",k
			self.setMotorRotationSpeed(k*self.speed, k*self.speed)

		
	#works for following left wall
	def followWallWithSonar(self, distance):
		while True:
			usReading = self.readUsSensor(self.usSensorBuffer)
			error = usReading - distance
			if error > 30:
				error = 30
			if error < -30:
				error = -30
			k = 0.1
			speed_right = self.speed + ((k/2)*(error))
			speed_left = self.speed - ((k/2)*(error))
			self.setMotorRotationSpeed(speed_left, speed_right)

# End of Robot Class

# main
robot = Robot()
#robot.MoveForwardsWithSonar(30)
robot.followWallWithSonar(30)
interface.terminate()
#END
