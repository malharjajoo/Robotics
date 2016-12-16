"""                                                                                                                           

FindBottles():
        SearchNavigateToWaypoint(10,30) //Or SearchArea(C)
        NavigateToWaypoint(84, 110)
        SearchNavigateToWaypoint(158, 110) //Or SearchArea(B)
        NavigateToWaypoint(110, 84)
        SearchNavigateToWaypoint(110, 10) //Or SearchArea(A)
        NavigateToWaypoint(<waypoint1>)
        Celebrate



SearchNavigateToWaypoint(self, x, y):
        Rotate to face waypoint (should leave the robot at 0, 90, 180, or 270 degrees)
        Rotate sonar to face datum wall (easily done with an if-else covering every start and end direction, since      we only have 4 possible states for each)
        Calculate expected distance (from datum wall)
        While !reached
                MoveForwardWithScan(min(distanceToWaypoint, 20cm), distanceToWall)
                if there is an anomaly
                        BumpNavigateToWaypoint(self.x, self.y + distance to anomaly)
                        return
                decrement remaining distance by distance moved and repeat
                if remaining distance == 0
                        break while loop

"""

# input - obstacle coordiantes
def BumpIntoObstacle(self,x,y):

            b = math.sqrt((self.x-x)*(self.x-x) + (self.y-y)*(self.y-y))

	   # move a slightly greter distance in order to bump into the obstacle.	
	    b = b + 3

            #while b > 0:
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

                #if b > 20:
                   # b = b-20
                    #self.moveForwards(20)
                #else :
                   # self.moveForwards(b)
                    #b = 0
            
	    bumped = self.moveForwards(b)	
	    return bumped





# ===============================SideWall Algorithm==========================


# Start Reading Here --- Baron

# As of now this function only deals with Area C
# outermost function
    def findObstacles(self):

	hitCount = 0 

#============== Area C =========================

	# preparation for area C
	self.navigateToWaypoint(74,30)
	self.rotateRight(90)

	#vrticla distance to wall = 168 ,we move 10 less than that 
	distance = (168-10) - self.y

	
	objectFound = self.moveForwardsWithScanning(distance)
	
	# either bumped into it while scanning or found it and then bumped into it.
	if(objectFound == True):
		print("incrementing hitCount....")

		hitCount += 1

	# Imp - need to decide what to do in this case. Not so obvious
	# Best strategy is to have a specific case for each Area A,B and C
	else:
		print("No object found in the side scan!")
		#navigateToWaypoint(a,b)
#===================================================

	#== Area B #=====
	# preparation for area B
	# Need to be careful with bumping into wall c in MCL map
	
	# 126 is the vertical location of point C in MCL map
	if(self.y > 126 ):
        	self.navigateToWaypoint(74,30)
        	self.navigateToWayPoint(100,84)

	else:
		self.navigateToWayPoint(100,84)

	#vrticla distance to wall d on mcl map
        distance = (210-10) - self.y

	#rotate sonar to face right
	self.rotateSonar(self,-90)
        objectFound = self.moveForwardsWithScanning(distance)

        # either bumped into it while scanning or found it and then bumped into it.
        if(objectFound == True):
                print("incrementing hitCount....")

                hitCount += 1

        # Imp - need to decide what to do in this case. Not so obvious
        # Best strategy is to have a specific case for each Area A,B and C
        else:
                print("No object found in the side scan!")
                #navigateToWaypoint(a,b)



#============== Area A =========================

        # preparation for area A
        self.navigateToWaypoint(90,12)
	#shortcut to turn it straight , facing towards wall d
	# (instead of finding orientation of robot to turn it straight)
        self.navigateToWayPoint(90,15)

        #vrticla distance to wall f = 84 ,we move 10 less than that 
        distance = (84-10) - self.y

	self.rotateSonar(self,-90)
        objectFound = self.moveForwardsWithScanning(distance)

        # either bumped into it while scanning or found it and then bumped into it.
        if(objectFound == True):
                print("incrementing hitCount....")

                hitCount += 1

        # Imp - need to decide what to do in this case. Not so obvious
        # Best strategy is to have a specific case for each Area A,B and C
        else:
                print("No object found in the side scan!")
                #navigateToWaypoint(a,b)
#===================================================
	
	if(hitCount == 3):
		self.navigateToWayPoint(84,30)
		print("Challeneg Completed")


		
    
    # this function is called until all 3 objects have not been hit once.
    def moveForwardsWithScanning(self, distance):
    #returns true if hit bottle,
    #returns result of Bump
    #returns false if reached end
        angle = self.distToAngle(distance)
        #set motor params
        interface.setMotorAngleControllerParameters(self.motors[0],motorParams)
        interface.setMotorAngleControllerParameters(self.motors[1],motorParams)
        interface.increaseMotorAngleReferences(self.motors,[angle,angle])
        startAngle = interface.getMotorAngles(self.motors)[0][0]
        
        #Before setting off, take five values of the sonar to the wall to get a good median
        for i in range(5):
             readUsSensor(self.usSensorBuffer)

        wallDistance  = self.readUsSensor(self.usSensorBuffer)
        while not interface.motorAngleReferencesReached(self.motors) :
        
                    #bumped into the obstacle accidentally while moving straight.
                if self.checkSensors(self.touch_ports[0]) or self.checkSensors(self.touch_ports[1]):
                    motorAngles = interface.getMotorAngles(self.motors)[0][0]
                    angleDiff = float(motorAngle-startAngle)
                    distanceTravelled = (angleDiff/float(angle)) * distance
                    

                    #stop motors
                    interface.setMotorPwm(self.motors[0],0)
                    interface.setMotorPwm(self.motors[1],0)

                    # need to rotate sonar to point forward for MCl to work
                    currentSonarAngle = interface.getMotorAngle(self.sonar_port[0][0])

                    if(currentSonarAngle > self.sonarStartingAngle):
                        self.rotateSonar(-90)
                    else:
                        self.rotateSonar(90)

		   # orientation of robot will not have changed since we moved straight only.
                    self.updatePosition(distanceTravelled, 0)

                    return true #hit bottle

		# found the object
		# then move towards it to bump into it.
                else:
			
		    bumped = False

                    usReading = self.readUsSensor(self.usSensorBuffer)
                    if (usReading - wallDistance) > 10:
			print("Found Obstacle!")
                        motorAngles = interface.getMotorAngles(self.motors)[0][0]
                        angleDiff = float(motorAngle-startAngle)
                        distanceTravelled = (angleDiff/float(angle)) * distance
                         #stop motors
                        interface.setMotorPwm(self.motors[0],0)
                        interface.setMotorPwm(self.motors[1],0)
                        
			# Major assumption - Increasing angles causes sonar to rotate left
			# need to rotate sonar to point forward for MCL to work
			if(self.currentSonarAngle > self.sonarStartingAngle ):
				print("Rotating sonar right")
				self.rotateSonar(90)

			elif(self.currentSonarAngle < self.sonarStartingAngle ):
				print("rotating sonar left")
                                self.rotateSonar(-90)  

			#Again , like above , we have moved only vertically
			self.updatePosition(distanceTravelled, 0)

			while(bumped!=True):	
				bumped = self.BumpIntoObstacle(usReading,self.y)
			#bumped into object
			return True

	# readched the end of the wall but did not find the object.

	return false




 # use position control for rotating sonar.
     # Calibrated it now.
     # sonar motor port = 3        
     def rotateSonar(self,angle):   
         #soar_motor_port = [3,0]
         
         angle = self.rotAngleToSonarMotorAngle(rotAngle)
         self.increaseMotorSonarAngle(angle):
             
     # calibration 
    def rotAngleToMotorAngle(angle):
        motorAngle = float(angle*2.3)/90
        return motorAngle

























































































































































Cl to work
                        currentSonarAngle = interface.getMotorAngle(self.sonar_port[0][0])
                        
                        navigateLeft = False;
                        if(currentSonarAngle > self.sonarStartingAngle):
                            self.rotateSonar(-90)
                            naviagateLeft = True
                        else:
                            self.rotateSonar(90)

                        self.updatePosition(distanceTravelled, 0)
                        
                        if navigateLeft:
                            return self.BumpNavigateToWaypoint(self.x - usReading,self.y)
                        else:
                            return self.BumpNavigateToWaypoint(self.x + usReading,self.y)

    
               
     # use position control for rotating sonar.
     # Calibrated it now.
     # sonar motor port = 3        
     def rotateSonar(self,angle):   
         #soar_motor_port = [3,0]
         print("Rotating sonar right")
         angle = self.rotAngleToSonarMotorAngle(rotAngle)
         self.increaseMotorSonarAngle(self, angle):
             
     # calibration 
    def rotAngle2MotorAngle(angle):
        motorAngle = float(motorAngle*2.3)/90
        return motorAngle
    
                                                                                                                           

FindBottles():
        SearchNavigateToWaypoint(10,30) //Or SearchArea(C)
        NavigateToWaypoint(84, 110)
        SearchNavigateToWaypoint(158, 110) //Or SearchArea(B)
        NavigateToWaypoint(110, 84)
        SearchNavigateToWaypoint(110, 10) //Or SearchArea(A)
        NavigateToWaypoint(<waypoint1>)
        Celebrate!

MoveForwardWithScan(self, distance, expected distance):
        Move distance cm forward
        While moving
                take us readings
                if it is >5cm different from expected distance
                        break and return the anomaly

SearchNavigateToWaypoint(self, x, y):
        Rotate to face waypoint (should leave the robot at 0, 90, 180, or 270 degrees)
        Rotate sonar to face datum wall (easily done with an if-else covering every start and end direction, since      we only have 4 possible states for each)
        Calculate expected distance (from datum wall)
        While !reached
                MoveForwardWithScan(min(distanceToWaypoint, 20cm), distanceToWall)
                if there is an anomaly
                        BumpNavigateToWaypoint(self.x, self.y + distance to anomaly)
                        return
                decrement remaining distance by distance moved and repeat
                if remaining distance == 0
                        break while loop

BumpNavigateToWaypoint(self, x, y):
        calculate angle
        calculate distance
        rotate to angle
        MoveForwards(distance + 5cm)
        if bumper is touching something
                increment bottles found
                return
        return to search line







# ===============================SideWall Algorithm==========================

    
    # this function is called until all 3 objects have not been hit once.
    def MoveForwardsWithScanning(self, distance, expectedDistance):
    #returns true if hit bottle,
    #returns result of BumpNavigateToWaypoint
    #returns false if reached end
        angle = self.distToAngle(distance)
        #set motor params
        interface.setMotorAngleControllerParameters(self.motors[0],motorParams)
            interface.setMotorAngleControllerParameters(self.motors[1],motorParams)
        interface.increaseMotorAngleReferences(self.motors,[angle,angle])
        startAngle = interface.getMotorAngles(self.motors)[0][0]
        
        #Before setting off, take five values of the sonar to the wall to get a good median
        for i in range(5):
            usReading = readUsSensor(self.usSensorBuffer)
            
        while not interface.motorAngleReferencesReached(self.motors) :
        
                    #bumped into the obstacle.
if self.checkSensors(self.touch_ports[0]) or self.checkSensors(self.touch_ports[1]):
                        motorAngles = interface.getMotorAngles(self.motors)[0][0]
                        angleDiff = float(motorAngle-startAngle)
                        distanceTravelled = (angleDiff/float(angle)) * distance
                        hitCount += 1
                        
                        #stop motors
                        interface.setMotorPwm(self.motors[0],0)
                        interface.setMotorPwm(self.motors[1],0)
                        
                        # need to rotate sonar to point forward for MCl to work
                        currentSonarAngle = interface.getMotorAngle(self.sonar_port[0][0])
                        
                        if(currentSonarAngle > self.sonarStartingAngle):
                            self.rotateSonar(-90)
                        else:
                            self.rotateSonar(90)
                            
                        self.updatePosition(distanceTravelled, 0)

                        return true #hit bottle
                        
                  else:
                      usReading = self.readUsSensor(self.useSensorBuffer)                     
                                    

    
               
     # use position control for rotating sonar.
     # Calibrated it now.
     # sonar motor port = 3        
     def rotateSonar(self,angle):   
         #soar_motor_port = [3,0]
         print("Rotating sonar right")
         angle = self.rotAngleToSonarMotorAngle(rotAngle)
         self.increaseMotorSonarAngle(self, angle):
             
     # calibration 
    def rotAngle2MotorAngle(angle):
        motorAngle = float(motorAngle*2.3)/90
        return motorAngle
    



# ===============================End of Robot Class =================
