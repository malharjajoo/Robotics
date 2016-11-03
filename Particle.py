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
		e = random.gauss(0,1)
		f =  random.gauss(0,1)
		self.x = self.x + ((distance + e)*math.cos(self.theta))
		self.y = self.y + ((distance + e)*math.sin(self.theta))
		self.theta = self.theta + f
		
	def updateAngleRandom(self, angle):
		g =  random.gauss(0,1)
		self.theta = self.theta + angle + g
