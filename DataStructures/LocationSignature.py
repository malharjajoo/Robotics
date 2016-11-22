import math
import time

# Location signature class: stores a signature characterizing one location
class LocationSignature:
        def __init__(self, no_bins = 360):
                self.sig = []
		for i in range(no_bins):
			self.sig.append(0)

        def print_signature(self):
                for i in range(len(self.sig)):
                        print self.sig[i]

	# draws it oin the graphcal web interface
	def draw(self,startpoint_x = 180,startpoint_y = 180):
		print("drawing now...")
		drawScale = 3    # Used to scale the particle positions on the screen
		
		
		#map.draw(origin, drawScale)
		
		for angle in range(len(self.sig)):
			depth = self.sig[angle]
		

			endpoint_x = startpoint_x + ( depth*math.cos(math.radians(angle)) )
			endpoint_y = startpoint_y + ( depth*math.sin(math.radians(angle)) )

			print "drawLine:" + str(((startpoint_x*drawScale),(startpoint_y*drawScale ),
                        (endpoint_x*drawScale),( endpoint_y*drawScale) ))
			

			time.sleep(0.01)


		
	# Returns the squared error
        def squared_error(self,locationSignature):
                sum = 0
                for i in range(len(self.sig)):
                        sum += (self.sig[i]-locationSignature.sig[i])*(self.sig[i]- locationSignature[i])

                return sum

	# returns deoth histogram
	def convertToDepthHistogram(self):
		dict = {}

		for depth in self.sig:	
			dict[depth] = dict.get(depth,0)+1

		return dict
		
