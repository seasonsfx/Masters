import sys

EPSILON = 0.01

def drange(start, stop, step):
	r = start
	while stop - r > 0.000001:
		yield r
		r += step

# define region
# define density
class sphere(object):
	def __init__(self, rad, x, y):
		self.radius = rad;
		self.x = x
		self.y = y

	def has_support(self, x, y, error):
		is_on_surface = abs((x-self.x)**2 + (y-self.y)**2) < self.radius**2
		return is_on_surface


def toFile(points, dim,  filename):
	f = open(filename, 'w')
	f.write("%d %d\n" % dim)
	f.write("""0 0 0
1 0 0
0 1 0
0 0 1
1 0 0 0
0 1 0 0
0 0 1 0
0 0 0 1
""")
	for point in points:
		f.write("%f %f %f %f\n" % (point[0], point[1], point[2], 0))


def discretise(scan_range, step, object_list):
	points = []
	count = 0
	for x in drange(scan_range['x'][0], scan_range['x'][1], step):
		count = count + 1

		for y in drange(scan_range['y'][0], scan_range['y'][1], step):
			
			support = False
			for obj in object_list:
				if(obj.has_support(x, y, EPSILON)):
					points.append((round(x,4), round(y,4), 0, 0.5))
					support = True
					break # one object is enough
			if not support:
				points.append((0, 0, 0, 0.5))
	#print count

	return points

def factor(num):
	n = num -1
	while n > 1:
		if num%n == 0:
			return (num/n, n)
		n = n-1
	return 0

def main():
	points = []
	scan_range = {
					'x': [0.0, 4.0],
					'y': [0.0, 4.0],
				}

	object_list = [sphere(1, 1, 2), sphere(1, 3, 2)]
	step = 0.1

	points = discretise(scan_range, step, object_list)

	#print points

	n = len(points)

	assert(n != 0)

	#dim =  factor(n)
	dim = (int((scan_range['x'][1] - scan_range['x'][0])/step), int((scan_range['y'][1] - scan_range['y'][0])/step) )
	print dim

	toFile(points, dim,  "out.ptx")

if __name__ == '__main__':
	main()
