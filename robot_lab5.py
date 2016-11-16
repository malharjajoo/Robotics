import brickpi
import random
import time
import CircularBuffer
import Particle
import math
import Map
import copy 

interface = None

class Robot:
	# attributes - ideally different components (motors, ultrasonic sensor, etc)
	motors = [0,1]
	speed = 6.0
	touch_ports = [0,1]
	sonar_port = 3
	usSensorBuffer = CircularBuffer.CircularBuffer()
	noOfParticles = 100
	particles_list = []
	theta = 0

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
		motorParams.minPWM = 8.0
		motorParams.pidParameters.minOutput = -255
		motorParams.pidParameters.maxOutput = 255
		#position ctrl: 517, 1000, 13
		#velocity ctrl: 100, 0, 0
		motorParams.pidParameters.k_p = 517
		motorParams.pidParameters.k_i = 1000
		motorParams.pidParameters.k_d = 13

		interface.setMotorAngleControllerParameters(self.motors[0],motorParams)
		interface.setMotorAngleControllerParameters(self.motors[1],motorParams)    

		self.createParticlesList()
		#temp draw debug, remove after
		self.printParticles()
		time.sleep(0.5)
		#self.updatePosition(160,0)
		#self.updatePosition(0,90)
		#self.updatePosition(34,0)
			
	def readWayPoints(self,filename):
		f = open(filename)
		input = f.read()

		pointList = list(map(int,input.split()))
		
		n = pointList[0]
		self.x = pointList[1]
		self.y = pointList[2]

		pointList = pointList[3:]

		pointList = list(zip(pointList[0:2*n:2],pointList[1:2*n:2]))
		for a,b in pointList:
			print ("next waypoint: " + str(a) + "," + str(b))
			while self.getDistance(a,b) > 1.0:
				self.navigateToWaypoint(a,b)
		
		
	def getDistance(self,x,y):
		return math.sqrt(float((self.x-x)*(self.x-x)) + float((self.y-y)*(self.y-y)))
	
	def createParticlesList(self):
		for i in range(0,self.noOfParticles):
			self.particles_list.append(Particle.Particle())
			self.particles_list[i].x = 84
			self.particles_list[i].y = 30
			self.particles_list[i].theta = 0
			self.particles_list[i].weight = 1.0/float(self.noOfParticles)

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
			self.updatePosition(distance, 0)

	def moveBackwards(self, distance=-1):
		if distance<0:
			self.setMotorRotationSpeed(-self.speed, -self.speed)
			while True:
				# sensor checks here
				time.sleep(1)    
		else:
			angle = self.distToAngle(-distance)
			self.increaseMotorAngle(angle, angle)
			self.updatePosition(-distance, 0)

	def rotateRight(self, rotAngle):
		print("NOW ROTATING RIGHT")
		angle = self.rotAngleToMotorAngle(rotAngle)
		self.increaseMotorAngle(angle, -angle)
		self.updatePosition(0,-rotAngle)

	def rotateLeft(self, rotAngle):
		print ("NOW ROTATING LEFT")
		angle = self.rotAngleToMotorAngle(rotAngle)
		self.increaseMotorAngle(-angle, angle)
		self.updatePosition(0, rotAngle)


	# conversion functions
	def distToAngle(self, dist):
		#41.5 - w/ wheels
		#41 with bumper
		angle = float(dist * 15.0)/40 
		return angle
	
	def rotAngleToMotorAngle(self, rotationAngle):
		#4.55 - w/ wheels
		#4.85 - w/ rear bumper
		return float(rotationAngle * 4.85) / 90.0
		

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
		motorParams.minPWM = 8.0
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


	def readUsSensor(self, circularBuffer):
		#first fill up the circular buffer if it isn't already
		usReading = interface.getSensorValue(self.sonar_port)
		if usReading:
			print(usReading)
		else:
			print "Failed to get Sonar reading."
		circularBuffer.add(usReading[0])
		
		return circularBuffer.getMedian()

	def MoveForwardsWithSonar(self, safeDistance):
		while True:
			usReading = self.readUsSensor(self.usSensorBuffer)    #returns the median of the circular buffer
			#print "median=",usReading
			error = usReading - safeDistance
			k = float(error)/30.0    #k gain - adjust for different smoothness
			if k > 1:
				k = 1
			if k < -1:
				k = -1
			#print "k=",k
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

	def updateParticlePositions(self, distance, angle):
		for particle in self.particles_list:
			if(distance!=0):
				particle.updateDistanceRandom(distance)
			if(angle!=0):
				particle.updateAngleRandom(angle)

	def printParticles(self, justList = False):
		if justList == False:
			drawScale = 3    # Used to scale the particle positions on the screen
			origin = (10,250)
			Map.map.draw(origin, drawScale)
			#draw origin
			oLen = 5
			print "drawLine:" + str(((origin[0]-oLen)*drawScale,origin[1]*drawScale,
				(origin[0]+oLen)*drawScale,origin[1]*drawScale))
			print "drawLine:" + str((origin[0]*drawScale,(origin[1]-oLen)*drawScale,
				origin[0]*drawScale,(origin[1]+oLen)*drawScale))

			p = []
			for particle in self.particles_list:
				p.append(((origin[0]+particle.x)*drawScale,(origin[1]-particle.y)*drawScale,-particle.theta))

			print "drawParticles:" + str(p)
		p = []
		for particle in self.particles_list:
			p.append((particle.x,particle.y,particle.theta))
		#print p
		time.sleep(1)
				 
	def moveSquare40Stop10(self):
		for i in range(0,4):
			for i in range(0,4):
				self.moveForwards(10)
			self.rotateLeft(90)

	def updatePosition(self, distance, angle):
	
		#for p in self.particles_list:
			#print((p.x,p.y,p.theta))
		self.updateParticlePositions(distance, angle)
		
		
		self.updateWeights()
#		for p in self.particles_list:
#			print((p.x,p.y,p.theta))
		self.printParticles()
		x_sum = 0
		y_sum = 0
		theta_sum = 0

		for particle in self.particles_list:
			x_sum += particle.x*particle.weight
			y_sum += particle.y*particle.weight
			theta_sum += particle.theta*particle.weight

		self.x = x_sum 
		self.y = y_sum 
		self.theta = theta_sum
		print("CURRENT ROBOT POS: " + str((x_sum, y_sum, theta_sum)))		

	def navigateToWaypoint(self, x, y):
			b = math.sqrt((self.x-x)*(self.x-x) + (self.y-y)*(self.y-y))

			while b > 0:
                                       


				new_x=x-self.x
				new_y=y-self.y
				rel_angle=math.degrees(math.atan2(float(new_y), float(new_x)))

			
				theta=self.theta%360

				newAngle=rel_angle-theta
			
			
				if (newAngle>=-180) and (newAngle<0):
					self.rotateRight(abs(newAngle))
				

				elif (newAngle<180) and (newAngle>=0):
					self.rotateLeft(newAngle)

				
				elif (newAngle>=180) and (newAngle<360):
					self.rotateRight(360-newAngle)

	
				elif (newAngle<-180) and (newAngle>-360):
					self.rotateLeft(360+newAngle)

				if b > 20:
                                	b = b-20
                               		self.moveForwards(20)
				else :
					self.moveForwards(b)
					b = 0
                                
        			


				#b = math.sqrt((self.x-x)*(self.x-x) + (self.y-y)*(self.y-y))
				
					

	def calculate_likelihood(self,x, y, theta, z):
		m_list = []
		if z == 255:
			return -1
		for wall in Map.map.walls:
			#A = x1,y1; B = x2,y2
			#m: lecture 5 slide 18
			x1 = wall[0]
			y1 = wall[1]
			x2 = wall[2]
			y2 = wall[3]
			#print ("current wall:" + str(wall))
			if ((y2 == y1 and (theta % 180 == 0)) or (x2 == x1 and (theta % 180 == 90))):
				m = -1
			else:
				m = float( ((y2-y1)*(x1-x)) - ((x2 - x1)*(y1 - y)) ) / float( ((y2 - y1)*math.cos(math.radians(theta))) - ((x2 - x1)*math.sin(math.radians(theta))) )
			m_list.append(m)
		min_m = 9999
		min_m_index = 0
		for i in range(len(m_list)):
			if (m_list[i] >= 0 and m_list[i] < min_m):
				current_wall = Map.map.walls[i]
				x1 = current_wall[0]
				y1 = current_wall[1]
				x2 = current_wall[2]
				y2 = current_wall[3]
				intersect_x = x + m_list[i]*math.cos(math.radians(theta))
				intersect_y = y + m_list[i]*math.sin(math.radians(theta))
				# check if intersection is within wall region
				if (intersect_x >= min(x1, x2)) and (intersect_x <= max(x1, x2)):
					if( (intersect_y >= min(y1, y2)) and (intersect_y <= max(y1, y2)) ):
						min_m = m_list[i]
						min_m_index = i
		current_wall = Map.map.walls[min_m_index]
		#print("current wall =",current_wall)
		x1 = current_wall[0]
		y1 = current_wall[1]
		x2 = current_wall[2]
		y2 = current_wall[3]
		k = 100.0
		sigma_s = 1

		#p(z|m)
		prob = k*math.exp((-(z-min_m)*(z-min_m))/(2*sigma_s*sigma_s))
		angle_of_incidence = math.degrees(
			math.acos(
				(
					(math.cos(math.radians(theta)) * (y1 - y2))
					+ (math.sin(math.radians(theta))*(x2 - x1))
				)/(
					math.sqrt(
						((y1 - y2)*(y1 - y2))
						+ ((x2 - x1)*(x2 - x1))
					)
				)
			)
		)

		if angle_of_incidence < 49:
			return prob
		else:
			#print("angle of incidence greater than 49")
			return -1
			#return prob

	def printParticleWeights(self):
		for p in self.particles_list:
			print(p.weight)

	def updateWeights(self):
		for i in range(5):
			z = self.readUsSensor(self.usSensorBuffer)
		#print("Before updating...")
		#self.printParticles(True)
		error_count = 0
		particles_list_copy = copy.deepcopy(self.particles_list)

		for particle in self.particles_list:
			prob = self.calculate_likelihood(particle.x,particle.y,particle.theta, z)
			if prob == -1:
				#print("out of sonar calibration range")
				error_count += 1
			else:
				particle.weight = particle.weight * prob
				#print("particle_weight = "+str(particle.weight))

		if(error_count > self.noOfParticles/4):
			print("Too many errors. Using old weights.")
			self.particles_list = copy.deepcopy(particles_list_copy)
		#print("After updating weights:")
		#self.printParticles(True)
		#time.sleep(1)
		#self.printParticles()
		self.normaliseParticlesList()
		#print("particle weights before resampling: ")
                #self.printParticleWeights()
		self.resampleParticlesList()
		#print("particle weights after resampling: ")
               #self.printParticleWeights()

	#particle weights normalized so that they all add to 1
	def normaliseParticlesList(self):
		weightSum = 0 
		for particle in self.particles_list:
		#	print("adding weight..." + str(particle.weight))
			weightSum += particle.weight
	#	print("weightSum = "+str(weightSum))
		
		if weightSum != 0:
			for particle in self.particles_list:  
				particle.weight = float(particle.weight) /float(weightSum )
		else:
			for particle in self.particles_list:  
				particle.weight = 1/float(self.noOfParticles)

	def getCumulativeWeights(self):
		
		cumulative_weights = []

		sum = 0 
		for p in self.particles_list:
			sum += p.weight
			cumulative_weights.append(sum) 
		
		#print("last one of cumulative weight list should be 1=",cumulative_weights[self.noOfParticles-1])	
		
		return cumulative_weights

	# input - old particle weights
	# output - new particles with same normalized weight
	def resampleParticlesList(self):
		#print("Size of current particles list: " + str(len(self.particles_list)))
		
		cumulative_weights = self.getCumulativeWeights()
		
		new_weight = 1.0/float(self.noOfParticles)

		resampled_particles_list = []

		# fill the resampled list
		while len(resampled_particles_list) < self.noOfParticles:
			
			randomWeight = random.random()
			
			# find intersectiopn with cumulative array
			# and add to the resampled list
			for i in range(len(cumulative_weights)):
				if(randomWeight >= cumulative_weights[i-1] and  randomWeight <= cumulative_weights[i]):
					resampled_particles_list.append(Particle.Particle())
					j=len(resampled_particles_list) - 1
					resampled_particles_list[j].x = self.particles_list[i].x
					resampled_particles_list[j].y = self.particles_list[i].y
					resampled_particles_list[j].theta = self.particles_list[i].theta
					resampled_particles_list[j].weight = new_weight
					break


			
		# reassign the no. of particles 
		self.particles_list = copy.deepcopy(resampled_particles_list)
		#print("Size of resampled particles list: " + str(len(self.particles_list)))



# End of Robot Class

# main
robot = Robot()

#robot.moveForwards(10)

robot.readWayPoints("waypoints.txt")

#for i in range(0,4):

##	robot.rotateRight(90)

interface.terminate()
#END
