                                                                                                                           

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

# input - obstacle coordiantes
BumpNavigateToWaypoint(self, x, y):
        
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
                    usReading = self.readUsSensor(self.usSensorBuffer)
                    if usReading - expectedDistance > 10:
                        motorAngles = interface.getMotorAngles(self.motors)[0][0]
                        angleDiff = float(motorAngle-startAngle)
                        distanceTravelled = (angleDiff/float(angle)) * distance
                         #stop motors
                        interface.setMotorPwm(self.motors[0],0)
                        interface.setMotorPwm(self.motors[1],0)
                        # need to rotate sonar to point forward for M






































































































































































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
