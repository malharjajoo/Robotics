import sys

class CircularBuffer:
	
	def __init__(self,maxSize = 15):
		self.circularBuffer = [0]* maxSize
		self.maxSize = maxSize
		
	def add(self,val):
		if len(self.circularBuffer) > self.maxSize:
			self.circularBuffer.pop(0)
		self.circularBuffer.insert(len(self.circularBuffer),val)
		#self.circularBuffer.pop()
		#self.circularBuffer.insert( 0,val)
		
	def getMedian(self):
		sortedList = list(self.circularBuffer)
		sortedList.sort()
		median = sortedList[(len(sortedList)-1)/2]
		return median
	
		if(self.maxSize %2 == 0):
			mid_1 = (self.maxSize/2)
			mid_2= mid_1+1
		
			median =  float(self.circularBuffer[mid_1-1]+self.circularBuffer[mid_2-1])/2.0
		
			
			return median 

		else:
			mid =( self.maxSize+1)/2
		
			median =  self.circularBuffer[mid-1] 
			
			return median
	
