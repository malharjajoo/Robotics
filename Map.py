class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
		x1 = wall[0]
		y1 = wall[1]
		x2 = wall[2]
		y2 = wall[3]
		print "drawLine:" + str((x1,y1,x2,y2))

map = Map()

# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
map.add_wall((0,0,0,168));        # a
map.add_wall((0,168,84,168));     # b
map.add_wall((84,126,84,210));    # c
map.add_wall((84,210,168,210));   # d
map.add_wall((168,210,168,84));   # e
map.add_wall((168,84,210,84));    # f
map.add_wall((210,84,210,0));     # g
map.add_wall((210,0,0,0));        # h
map.draw();
