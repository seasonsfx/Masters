import sys

EPSILON = 0.01

def drange(start, stop, step):
	r = start
	while r < stop:
		yield r
		r += step

# define region
# define density
class sphere(object):
	def __init__(self, rad):
		self.radius = rad;

	def has_support(self, x, y, z, error):
		is_on_surface = abs(x**2 + y**2 + z**2 - self.radius**2) < error
		#is_on_surface = x**2 + y**2 + z**2 < self.radius**2
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
	for x in drange(scan_range['x'][0], scan_range['x'][1], step):
		for y in drange(scan_range['y'][0], scan_range['y'][1], step):
			for z in drange(scan_range['z'][0], scan_range['z'][1], step):
				for obj in object_list:
					if(obj.has_support(x, y, z, EPSILON)):
						points.append((round(x,4), round(y,4), round(z,4)))
						break # one object is enough

	return points

def factor(num):
	n = num -1
	while num > 1:
		if num%n == 0:
			return (num/n, n)
		n = n-1
	return 0

def main():
	points = []
	scan_range = {
					'x': [-4.0, 4.0],
					'y': [-4.0, 4.0],
					'z': [-4.0, 4.0],
				}

	object_list = [sphere(4.0)]
	step = 0.05

	points = discretise(scan_range, step, object_list)

	#print points

	n = len(points)

	assert(n != 0)

	dim =  factor(n)
	print dim

	toFile(points, dim,  "out.ptx")

if __name__ == '__main__':
	main()