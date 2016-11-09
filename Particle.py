import sys
import random
import math


class Particle:
	def __init__(self):
		self.x = 0 
		self.y = 0 
		self.theta = 0
		self.weight = 0.01

	def updateDistanceRandom(self, distance):
		#e = random.gauss(0,1) * (float(distance)/10.0)
		#f =  random.gauss(0,1) * (float(distance)/10.0)
		e = random.gauss(0, math.sqrt(float(distance)/15.0)) # change divisor to change spreading
		f = random.gauss(0, math.sqrt(float(distance)/15.0)) # change divisor to change spreading
		#print("old x is "+str(self.x))
		#print("old y is "+str(self.y))
		self.x = self.x + ((distance + e)*math.cos(math.radians(self.theta)))
		self.y = self.y + ((distance + e)*math.sin(math.radians(self.theta)))
		#print("new x is "+str(self.x))
		#print("new y x is "+str(self.y))
		self.theta = self.theta + f
		#if self.theta < 0:
		#	self.theta += 360
		#else:
		#	self.theta = self.theta % 360
		
	def updateAngleRandom(self, angle):
		g =  random.gauss(0,1) * (angle/180)
		#g = random.gauss(0, math.sqrt(float(angle)/180.0))
		self.theta = self.theta + angle + g
		
		#if self.theta < 0:
		#	self.theta += 360
		#else:
		#	self.theta = self.theta % 360
