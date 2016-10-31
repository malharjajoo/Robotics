import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]
speed = 12.0

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0
motorParams.minPWM = 18.0
motorParams.pidParameters.minOutput = -255
motorParams.pidParameters.maxOutput = 255
# 517, 1000, 13
motorParams.pidParameters.k_p = 100
motorParams.pidParameters.k_i = 0
motorParams.pidParameters.k_d = 0

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

interface.setMotorRotationSpeedReferences(motors,[speed,speed])

print "Press Ctrl+C to exit"
while True:
	time.sleep(10)

interface.terminate()
