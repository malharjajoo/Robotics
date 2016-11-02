import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]
speed = 6.0

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 5.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
# 517, 1000, 13
motorParams.pidParameters.k_p = 150
motorParams.pidParameters.k_i = 20
motorParams.pidParameters.k_d = 10


interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

interface.setMotorRotationSpeedReferences(motors,[speed,speed])

print "Press Ctrl+C to exit"
while True:
	time.sleep(10)

interface.terminate()
