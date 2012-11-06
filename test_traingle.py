#!/usr/bin/python
import numpy as np
from numpy import array

import matplotlib.pyplot as plt
import matplotlib.patches as patches 
import matplotlib.tri as tri
import math



def draw(triangle, point):
	poly = patches.Polygon(triangle, fill=False)

	fig = plt.figure() 
	ax = fig.add_subplot(111) 
	ax.set_xlim(-4,4) 
	ax.set_ylim(-4,4)
	ax.add_patch(poly)

	poly2 = patches.Circle(point, 0.05)
	ax.add_patch(poly2)

	plt.show()

def side(p, a, b):
	vec1 = p - a
	vec2 = b - a

	return np.cross(vec1, vec2)

def inside(point, triangle):
	sides = [side(point, triangle[0], triangle[1]) > 0, side(point, triangle[1], triangle[2]) > 0, side(point, triangle[2], triangle[0]) > 0]

	if sides[0] == sides[1] == sides[2]:
		return true
	else:
		return outside

def main():
	triangle = [array([3, 1]), array([1,2]), array([1,3])]
	
	point = array([0, 1.8])

	#point = triangle[2]


	# rebase the triangle abc
	vec1 = -(triangle[0] - triangle[1])
	vec2 = -(triangle[0] - triangle[2])

	mat = np.hstack((vec1.reshape(2,1), vec2.reshape(2,1)))
	print "vec1 ", vec1
	print "vec2", vec2
	print "mat", mat

	invmat = np.linalg.inv(mat)

	rb = invmat.dot(point-triangle[0])

	print point
	print rb

	print "-"*8


	if rb[0] + rb[1] < 1 and rb[0] > 0 and rb[1] > 0:
		print "inside"

	if rb[0] + rb[1] >= 1 and rb[0] > 0 and rb[1] > 0:
		print "closest to [1,2]"

	if rb[0] + rb[1] >= 1 and rb[0] <= 0 and rb[1] > 0:
		print "closest to 2"

	if rb[0] + rb[1] >= 1 and rb[1] < 0:
		print "closest to 1"

	if rb[0] + rb[1] < 1 and rb[0] <= 0 and rb[1] > 0:
		print "closest to [0,2]"

	if rb[0] < 0 and rb[1] < 0:
		print "closest to 0"

	if rb[0] + rb[1] < 1 and rb[0] > 0 and rb[1] < 0:
		print "closest to [0, 1]"

	

	draw(triangle, point)


if __name__ == "__main__":
	main()