import sys
import random
import math


class Particle:
	def __init__(self, x_=0, y_=0,theta_=0,weight_=0.01):
		self.x = x_ 
		self.y = y_ 
		self.theta = theta_
		self.weight = weight_

	def updateDistanceRandom(self, distance):
		#e = random.gauss(0,1) * (float(distance)/10.0)
		#f =  random.gauss(0,1) * (float(distance)/10.0)
		e = random.gauss(0, math.sqrt(float(abs(distance))/15.0)) # change divisor to change spreading
		f = random.gauss(0, math.sqrt(float(abs(distance))/15.0)) # change divisor to change spreading
		#print("old x is "+str(self.x))
		#print("old y is "+str(self.y))
		#print("new particle: "+ str(hex(id(self))))
		#print("before: ")
                #print((self.x,self.y,self.theta))
		#print ("x += :"+ str((distance + e)*math.cos(math.radians(self.theta))))
		self.x = self.x + ((distance + e)*math.cos(math.radians(self.theta)))
		self.y = self.y + ((distance + e)*math.sin(math.radians(self.theta)))
		#print("after: ")
                #print((self.x,self.y,self.theta))

		#print("new x is "+str(self.x))
		#print("new y x is "+str(self.y))
		
		self.theta = self.theta + f
		#if self.theta < 0:
		#	self.theta += 360
		#else:
		#	self.theta = self.theta % 360
		
	def updateAngleRandom(self, angle):
		#g =  random.gauss(0,1) *( float(angle)/360.0)
		#print("g = " + str(g))
		g = random.gauss(0, math.sqrt(float(abs(angle))/90.0))
		self.theta = self.theta + angle  + g
		print "angle = " + str(angle)
		#if self.theta < 0:
		#	self.theta += 360
		#else:
		#	self.theta = self.theta % 360
