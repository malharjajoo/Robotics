import sys

class CircularBuffer:
	
	def __init__(self,maxSize):
		self.circularBuffer = [0]*maxSize
		self.maxSize = maxSize
		
	def add(self,val):
		self.circularBuffer.pop()
		self.circularBuffer.insert( 0,val)
		
	def getMedian(self):
		
		

		if(self.maxSize %2 == 0):
			mid_1 = (self.maxSize/2)
			mid_2= mid_1+1
		
			median =  float(self.circularBuffer[mid_1-1]+self.circularBuffer[mid_2-1])/2.0
		
			
			return median 

		else:
			mid =( self.maxSize+1)/2
		
			median =  self.circularBuffer[mid-1] 
			
			return median


	
	
