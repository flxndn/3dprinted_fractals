import math
import numpy

from inspect import getmembers
from pprint import pprint
#from vpython import * # vector elements, cylinder, sphere

#-------------------------------------------------------------------------------
class Point:
#-------------------------------------------------------------------------------
	#---------------------------------------------------------------------------
	def __init__(self,x,y,z):
	#---------------------------------------------------------------------------
		self.x = x
		self.y = y
		self.z = z
	#---------------------------------------------------------------------------
	def __str__(self):
	#---------------------------------------------------------------------------
		return "[{0}, {1}, {2}]".format(self.x, self.y, self.z)
#-------------------------------------------------------------------------------
class _3DPrintable:
#-------------------------------------------------------------------------------
	#---------------------------------------------------------------------------
	def toSCAD(self):
	#---------------------------------------------------------------------------
		raise NotImplementedError('subclasses must override toSCAD()!')
#-------------------------------------------------------------------------------
class Transform:
#-------------------------------------------------------------------------------
	# matrix t
	#---------------------------------------------------------------------------
	def apply(self, point):
	#---------------------------------------------------------------------------
		x= self.t[0,0]*point.x + self.t[0,1]*point.y + self.t[0,2]*point.z + self.t[0,3]
		y= self.t[1,0]*point.x + self.t[1,1]*point.y + self.t[1,2]*point.z + self.t[1,3]
		z= self.t[2,0]*point.x + self.t[2,1]*point.y + self.t[2,2]*point.z + self.t[2,3]
		return Point(x,y,z)
	#---------------------------------------------------------------------------
	def compose(self, transform):
	#---------------------------------------------------------------------------
		#print "// self\n"
		#print (self.t)

		#print "// transform\n"
		#print (transform.t)

		self.t = transform.t * self.t
		#print "// self 2\n"
		#print (self.t)
	#---------------------------------------------------------------------------
	def __str__(self):
	#---------------------------------------------------------------------------
		t = self.t
		return "[ [ {0}, {1}, {2}, {3}], [ {4}, {5}, {6}, {7}], [ {8}, {9}, {10}, {11}], [ {12}, {13}, {14}, {15}]]".format(
						t[0,0], t[0,1], t[0,2], t[0,3],
						t[1,0], t[1,1], t[1,2], t[1,3],
						t[2,0], t[2,1], t[2,2], t[2,3],
						t[3,0], t[3,1], t[3,2], t[3,3])
#-------------------------------------------------------------------------------
class TransformUnitaryXAxis(Transform):
#-------------------------------------------------------------------------------
	#---------------------------------------------------------------------------
	def __init__(self, origin, end, angle):
	#---------------------------------------------------------------------------
		print "// TransformUnitaryXAxis::angle: {0}".format(angle)
		difference = Point( end.x - origin.x, end.y - origin.y, end.z - origin.z) 
		modulus = math.sqrt( math.pow(difference.x,2) + math.pow(difference.y, 2) + math.pow(difference.z,2))
		print "// TransformUnitaryXAxis::modulus: {0}".format(modulus)
		if modulus < 1.0e-6 :
			raise

		modulus_base = math.sqrt( math.pow(difference.x,2) + math.pow(difference.y, 2) )
		print "// TransformUnitaryXAxis::modulus_base: {0}".format(modulus_base)
		print "// TransformUnitaryXAxis::difx: {0}, dify: {1}".format(difference.x, difference.y)
		if modulus_base < 1.0e-6:
			delta = math.pi/2
			fi = 0.0
		else:
			delta=math.acos(modulus_base/modulus)
			fi=math.atan2(difference.y,difference.x)
			print "// fi: {0}, dx: {1}, dy: {2}".format(fi, difference.x, difference.y)
		print "// fi: {0}, delta: {1}, angle: {2}, modulus: {3}".format(fi, delta, angle, modulus)
		#print 'u'
		#print Unity().t
		#print 'rz'
		#print RotateZ(fi)
		#print 'ry'
		#print RotateY(delta)
		#print 'rx'
		#print RotateX(angle)
		#print 'sc'
		#print Scale(modulus)
		#print 'd'
		#print Displacement(origin)
		self.t=Unity().t
		self.compose(RotateY(delta))
		self.compose(RotateZ(fi))
		self.compose(RotateX(angle))
		self.compose(Scale(modulus))
		self.compose(Displacement(origin))
#-------------------------------------------------------------------------------
class Scale(Transform):
#-------------------------------------------------------------------------------
	def __init__(self, scale):
	#---------------------------------------------------------------------------
		unity = Unity()
		self.t=unity.t

		self.t[0,0]=scale
		self.t[1,1]=scale
		self.t[2,2]=scale
#-------------------------------------------------------------------------------
class RotateX(Transform):
#-------------------------------------------------------------------------------
	def __init__(self, angle):
	#---------------------------------------------------------------------------
		unity = Unity()
		self.t=unity.t

		c = math.cos(angle)
		s = math.sin(angle)

		#print 'angle:{0}, c:{1} s{2}'.format(angle, c, s)

		self.t[1,1]=c
		self.t[1,2]=s
		self.t[2,1]=-s
		self.t[2,2]=c
		
		#print self
#-------------------------------------------------------------------------------
class RotateY(Transform):
#-------------------------------------------------------------------------------
	def __init__(self, angle):
	#---------------------------------------------------------------------------
		unity = Unity()
		self.t=unity.t

		c = math.cos(angle)
		s = math.sin(angle)

		self.t[0,0]=c
		self.t[0,2]=-s
		self.t[2,0]=s
		self.t[2,2]=c
#-------------------------------------------------------------------------------
class RotateZ(Transform):
#-------------------------------------------------------------------------------
	def __init__(self, angle):
	#---------------------------------------------------------------------------
		unity = Unity()
		self.t=unity.t

		c = math.cos(angle)
		s = math.sin(angle)

		self.t[0,0]=c
		self.t[0,1]=-s
		self.t[1,0]=s
		self.t[1,1]=c
#-------------------------------------------------------------------------------
class Displacement(Transform):
#-------------------------------------------------------------------------------
	def __init__(self, point):
	#---------------------------------------------------------------------------
		unity = Unity()
		self.t=unity.t
		self.t[0,3]=point.x
		self.t[1,3]=point.y
		self.t[2,3]=point.z
#-------------------------------------------------------------------------------
class Zero(Transform):
#-------------------------------------------------------------------------------
	def __init__(self):
	#---------------------------------------------------------------------------
		self.t=numpy.matrix([[0.0, 0.0, 0.0, 0.0],[ 0.0, 0.0, 0.0, 0.0],[ 0.0, 0.0, 0.0, 0.0],[ 0.0, 0.0, 0.0, 0.0]]);
#-------------------------------------------------------------------------------
class Unity(Transform):
#-------------------------------------------------------------------------------
	def __init__(self):
	#---------------------------------------------------------------------------
		self.t=numpy.matrix([[1.0, 0.0, 0.0, 0.0],[ 0.0, 1.0, 0.0, 0.0],[ 0.0, 0.0, 1.0, 0.0],[ 0.0, 0.0, 0.0, 1.0]]);
#-------------------------------------------------------------------------------
class Sphere(_3DPrintable):
#-------------------------------------------------------------------------------
	def __init__(self,center, radius):
	#---------------------------------------------------------------------------
		self.center = center
		self.radius = radius
	#---------------------------------------------------------------------------
	def toSCAD(self):
	#---------------------------------------------------------------------------
		ret =  'translate([{0},{1},{2}]){{ sphere({3},$fn=20);}}\n'.format( self.center.x,
																	self.center.y,
																	self.center.z,
																	self.radius )
		return ret
#-------------------------------------------------------------------------------
class Cylinder(_3DPrintable):
	#---------------------------------------------------------------------------
	def __init__(self,origin, end, radius):
	#---------------------------------------------------------------------------
		self.origin = origin
		self.end = end
		self.radius = radius
	#---------------------------------------------------------------------------
	def toSCAD(self):
	#---------------------------------------------------------------------------
		print "// {0}->{1}".format(self.origin,self.end)

		transform = TransformUnitaryXAxis(self.origin, self.end, 0)
		s = 'multmatrix(m={}){{rotate([0,90,0]){{cylinder(r={},center=false,$fn=20);}};}}\n'.format(str(transform),self.radius/10.0)
		return s

		# cylinder($fn = 0, $fa = 12.000000, $fs = 2.000000, h = 20.0, r1 = 2.0, r2 = 2.0, center = false);
#-------------------------------------------------------------------------------
class _3DGeometric(_3DPrintable):
#-------------------------------------------------------------------------------
	def vertices(self):
	#---------------------------------------------------------------------------
		raise NotImplementedError('subclasses must override toSCAD()!')
	#---------------------------------------------------------------------------
	def edges(self):
	#---------------------------------------------------------------------------
		raise NotImplementedError('subclasses must override toSCAD()!')
	#---------------------------------------------------------------------------
	def toSCAD(self):
	#---------------------------------------------------------------------------
		ret=""
		for vertex in self.vertices():
			sphere=Sphere(vertex, 1)
			ret = ret + sphere.toSCAD()

		contador=0
		for edge in self.edges():
			cylinder = Cylinder(edge[0], edge[1], 1)
			ret = ret + cylinder.toSCAD()

		return ret
#-------------------------------------------------------------------------------
# TODO: class Sierpinsky
# TODO: class Cube
#-------------------------------------------------------------------------------
class Tetrahedron(_3DGeometric):
#-------------------------------------------------------------------------------
	def __init__(self, first_point, second_point, rotation):
	#---------------------------------------------------------------------------
		#print "// Tetrahedron::rotation: {0}\n".format(rotation)
		self.v=[]
		self.v.append(first_point)
		self.v.append(second_point)
		transform = TransformUnitaryXAxis(first_point, second_point, rotation)
		#print '// transform: ' + str(transform)

		p3 = Point(0.5, math.sqrt(1.0-math.pow(0.5,2.0)), 0)
		#print '// p3: ' + str(p3)
		self.v.append(transform.apply(p3))

		p4 = Point(0.5, 0.5/math.sqrt(3.0), math.sqrt(2.0/3.0))
		#print '// p4: ' + str(p4)
		self.v.append(transform.apply(p4))
	#---------------------------------------------------------------------------
	def vertices(self):
	#---------------------------------------------------------------------------
		return self.v
	#---------------------------------------------------------------------------
	def edges(self):
	#---------------------------------------------------------------------------
		ret=[	[self.v[0], self.v[1]],
				[self.v[1], self.v[2]],
				[self.v[2], self.v[0]],
				[self.v[0], self.v[3]],
				[self.v[1], self.v[3]],
				[self.v[2], self.v[3]] ]
		return ret
#-------------------------------------------------------------------------------
class Scaffold(_3DGeometric):
#-------------------------------------------------------------------------------
	def __init__(self, first_point, second_point, rotation):
	#---------------------------------------------------------------------------
		raise NotImplementedError('subclasses must override __init__!')
	#---------------------------------------------------------------------------
	def vertices(self):
	#---------------------------------------------------------------------------
		return self.base.vertices()
	#---------------------------------------------------------------------------
	def edges(self):
	#---------------------------------------------------------------------------
		return self.base.edges()
	#---------------------------------------------------------------------------
	def __str__(self):
	#---------------------------------------------------------------------------
		s='['
		for vertex in self.vertices():
			s = s + vertex.__str__()
		s = s + "]"
		return s
#-------------------------------------------------------------------------------
class ScaffoldTetrahedron(Scaffold):
#-------------------------------------------------------------------------------
	def __init__(self, first_point, second_point, rotation):
	#---------------------------------------------------------------------------
		#print "// ScaffoldTetrahedron::rotation: {0}\n".format(rotation)

		self.base = Tetrahedron(first_point, second_point, rotation)
#-------------------------------------------------------------------------------

t1=ScaffoldTetrahedron(Point(0,0,0), Point(10,0,0), 0.00)
print t1.toSCAD()

#t2=ScaffoldTetrahedron(Point(10,0,0), Point(20,0,0), 0.00)
#print t2.toSCAD()

#t3=ScaffoldTetrahedron(t1.vertices()[2], t2.vertices()[2], 0.00)
#print t3.toSCAD()

#t4=ScaffoldTetrahedron(t1.vertices()[3], t2.vertices()[3], 0.00)
#print t4.toSCAD()

#c=Cylinder(Point(0, 0,0), Point(5.0, 2.88675134595, 8.16496580928), 1)
#print c.toSCAD()

