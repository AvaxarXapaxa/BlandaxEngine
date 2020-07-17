# Future note: I made this before but i abandoned it because it started to be complicated
# Now, i use some parts of the code

################################################################################
# VaxEngine: Low poly 3D engine for pygame; Written in Cython                  #
# Written by AvaxarXapaxa; github.com/AvaxarXapaxa; avaxar.itch.io             #
# Special thanks to the python and pygame community                            #
# The whole module was written in this single file because headers suck, lol   #
################################################################################

# NOTE: I probably use some weird/wrong terms here and ridiculous weird code but whatever, ;p;

# Do some initializing... [Requires: >= Python 3.7; >= Pygame 2.0.0dev10]
import sys, os, socket
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
print('Thanks for using VaxEngine and thanks to the pygame community for providing an easy python windowing framework!')
pygame.init()
from libcpp.vector cimport vector
cdef extern from "<math.h>":
	double sin(double)
	double cos(double)
	double tan(double)

# Math constants
cdef double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286
cdef double euler = 2.7182818284

# No radians!
cdef double dsin(double x):
	return sin(x*pi/180)
cdef double dcos(double x):
	return cos(x*pi/180)
cdef double dtan(double x):
	return tan(x*pi/180)
cdef double remap(double x, double amin, double amax, double bmin, double bmax):
	return (x - amin) * (bmax - bmin) / (amax - amin) + bmin

# Rotate vector 2 according to origin (0, 0)
cdef (double, double) rotateVec2((double, double) vec2, double angle):
	angle = -angle
	cdef double sZ = dsin(angle)
	cdef double cZ = dcos(angle)
	return (vec2[0] * cZ - vec2[1] * sZ, vec2[0] * sZ + vec2[1] * cZ)

# Rotate vector 3 according to origin (0, 0, 0)
# RotX: Yaw, horizontal rotate. 0 degree facing Z+
# RotY: Pitch, vertical rotate. Straight
# RotZ: Roll, rotate 2D
cdef (double, double, double) rotateVec3((double, double, double) vec3, (double, double, double) rot3):
	# NANI??? A thicc blocc of cdef
	cdef double rotX = rot3[0]
	cdef double rotY = rot3[1]
	cdef double rotZ = rot3[2]
	cdef double x = vec3[0]
	cdef double y = vec3[1]
	cdef double z = vec3[2]
	cdef double oldX = x
	cdef double oldY = y
	cdef double sX = dsin(rotX)
	cdef double sY = dsin(rotY)
	cdef double cX = dcos(rotX)
	cdef double cY = dcos(rotY)
	cdef double sZ = dsin(rotZ)
	cdef double cZ = dcos(rotZ)
	if not ((rotX % 360 == 0) and (rotY % 360 == 0)):
		# Do some rotation matrix witchcraft
		x = x * cX + z * sX
		z = z * cX - oldX * sX
		y = y * cY + z * sY
		z = z * cY - oldY * sY
		oldX = x
		oldY = y
	if not rotZ % 360 == 0:
		rotZ *= -1
		x = oldX * cZ - oldY * sZ
		y = oldX * sZ + oldY * cZ
	return (x, y, z)

# Rotate a std::vector full of vector 2s
cdef vector[(double, double)] rotateVec2s(vector[(double, double)] vec2s, double angle):
	angle = -angle
	cdef double sZ = dsin(angle)
	cdef double cZ = dcos(angle)
	cdef vector[(double, double)] ret
	cdef unsigned long int vec2 = 0
	for vec2 in range(vec2s.size()):
		ret.push_back((vec2s[vec2][0] * cZ - vec2s[vec2][1] * sZ, vec2s[vec2][0] * sZ + vec2s[vec2][1] * cZ))
	return ret

# Rotate a std::vector full of vector 3s
cdef vector[(double, double, double)] rotateVec3s(vector[(double, double, double)] vec3s, (double, double, double) rot3):
	# Cdefs of constants
	cdef double rotX = rot3[0]
	cdef double rotY = rot3[1]
	cdef double rotZ = rot3[2]*-1
	cdef double sX = dsin(rotX)
	cdef double sY = dsin(rotY)
	cdef double cX = dcos(rotX)
	cdef double cY = dcos(rotY)
	cdef double sZ = dsin(rotZ)
	cdef double cZ = dcos(rotZ)
	# For loop
	cdef unsigned long int vec3 = 0
	cdef double x = 0
	cdef double y = 0
	cdef double z = 0
	cdef double oldX = 0
	cdef double oldY = 0
	cdef vector[(double, double, double)] ret
	for vec3 in range(vec3s.size()):
		x = vec3s[vec3][0]; y = vec3s[vec3][1]; z = vec3s[vec3][2]; oldX = x; oldY = y
		if not ((rotX % 360 == 0) and (rotY % 360 == 0)):
			# Do some rotation matrix witchcraft, again
			x = x * cX + z * sX
			z = z * cX - oldX * sX
			y = y * cY + z * sY
			z = z * cY - oldY * sY
			oldX = x
			oldY = y
		if not rotZ % 360 == 0:
			x = oldX * cZ - oldY * sZ
			y = oldX * sZ + oldY * cZ
		ret.push_back((x, y, z))
	return ret

# Rotate vec3s manual trigonometry input
cdef (double, double, double) rotateVec3tri((double, double, double) vec3, double sX, double cX, double sY, double cY, double sZ, double cZ):
	cdef double x = vec3[0]
	cdef double y = vec3[1]
	cdef double z = vec3[2]
	cdef double oldX = x
	cdef double oldY = y
	if not ((sX == 0) and (sY == 0)):
		x = x * cX + z * sX
		z = z * cX - oldX * sX
		y = y * cY + z * sY
		z = z * cY - oldY * sY
		oldX = x
		oldY = y
	sZ *= -1
	cZ *= -1
	if not sZ == 0:
		x = oldX * cZ - oldY * sZ
		y = oldX * sZ + oldY * cZ
	return (x, y, z)

# Rotate a std::vector full of vector 3s !!! Manual trigonometry input
cdef vector[(double, double, double)] rotateVec3stri(vector[(double, double, double)] vec3s, double sX, double cX, double sY, double cY, double sZ, double cZ):
	# For loop
	cdef unsigned long int vec3 = 0
	cdef double x = 0
	cdef double y = 0
	cdef double z = 0
	cdef double oldX = 0
	cdef double oldY = 0
	cdef vector[(double, double, double)] ret
	for vec3 in range(vec3s.size()):
		x = vec3s[vec3][0]; y = vec3s[vec3][1]; z = vec3s[vec3][2]; oldX = x; oldY = y
		x = x * cX + z * sX
		z = z * cX - oldX * sX
		y = y * cY + z * sY
		z = z * cY - oldY * sY
		oldX = x
		oldY = y
		sZ *= -1
		cZ *= -1
		x = oldX * cZ - oldY * sZ
		y = oldX * sZ + oldY * cZ
		ret.push_back((x, y, z))
	return ret

# Clip behind Z 0
cdef ((double, double, double), (double, double, double), unsigned short int) clipLineBehind((double, double, double) pos1, (double, double, double) pos2):
	cdef double x1 = pos1[0]
	cdef double y1 = pos1[1]
	cdef double z1 = pos1[2]
	cdef double x2 = pos2[0]
	cdef double y2 = pos2[1]
	cdef double z2 = pos2[2]
	if z1 >= 0 and z2 >= 0:
		return ((x1, y1, z1), (x2, y2, z2), 0)
	if z1 < 0 and z2 < 0:
		return ((0, 0, 0), (0, 0, 0), 1) # Both points are behind Z 0
	if z1 == z2:
		if z1 < 0:
			z1 = 0; z2 = 0
		return ((x1, y1, z1), (x2, y2, z2), 0)
	if z1 < 0 and z2 >= 0:
		return ((remap(0, z1, z2, x1, x2), remap(0, z1, z2, y1, y2), 0.0), (x2, y2, z2), 0)
	if z1 >= 0 and z2 < 0:
		return ((x1, y1, z1), (remap(0, z1, z2, x1, x2), remap(0, z1, z2, y1, y2), 0.0), 0)

# Clip polygon behind Z 0
cdef vector[(double, double, double)] clipPolygonBehind(vector[(double, double, double)] poly):
	cdef vector[(double, double, double)] ret
	cdef unsigned int point
	cdef (double, double, double) prevPoint
	for point in range(poly.size()):
		if point == 0:
			prevPoint = poly[poly.size()-1]
		else:
			prevPoint = poly[point-1]

		if poly[point][2] < 0 and prevPoint[2] < 0:
			# The whole line is behind
			continue
		elif poly[point][2] < 0 and prevPoint[2] >= 0:
			# The current point is behind
			ret.push_back(prevPoint)
			ret.push_back(clipLineBehind(prevPoint, poly[point])[1])
		elif poly[point][2] >= 0 and prevPoint[2] < 0:
			# The previous point is behind
			ret.push_back(clipLineBehind(poly[point], prevPoint)[1])
			ret.push_back(poly[point])
		else:
			# The whole line isn't behind
			ret.push_back(prevPoint)
			ret.push_back(poly[point])
	cdef vector[(double, double, double)] retDupeFix
	cdef unsigned int ind
	cdef (double, double, double) prev
	for ind in range(ret.size()):
		if ind == 0:
			prev = ret[ret.size()-1]
		else:
			prev = ret[ind-1]
		if ret[ind] == prev:
			continue
		else:
			retDupeFix.push_back(ret[ind])
	return retDupeFix


cdef class Node:
	# Store polygons each axis position because Cython ctuple-nested vector bug
	cdef vector[vector[double]] Xverts, Yverts, Zverts
	cpdef double x, y, z, yaw, pitch, roll
	# Two word types causes an error on ctuples that is templated in a vector, I can't do vector[(unsigned int, unsigned int, unsigned int, unsigned it)]
	# And ctypedef'ing it works but if i try to assign it, it cannot compare it, like wth??
	cdef vector[unsigned char] polyVertsColorR, polyVertsColorG, polyVertsColorB, polyVertsColorA
	cdef unsigned char enabled
	# Variables for addPoly()
	cdef unsigned int _polyPoint
	cdef vector[double] _polyVecX, _polyVecY, _polyVecZ
	# Variables for getHierarchy()
	cdef unsigned int _childIter, _polyIter, _pointIter, _for
	cdef double _sX, _sY, _sZ, _cX, _cY, _cZ
	cdef (double, double, double) _vec3
	cdef (vector[vector[double]], vector[vector[double]], vector[vector[double]], vector[unsigned char], vector[unsigned char], vector[unsigned char], vector[unsigned char]) _res
	cdef (vector[vector[double]], vector[vector[double]], vector[vector[double]], vector[unsigned char], vector[unsigned char], vector[unsigned char], vector[unsigned char]) _childHie
	# Pure Python stuff... SPEED V -9000%
	cpdef list children
	cpdef object parent
	cpdef dict userArgs

	def __init__(self, x=0, y=0, z=0, yaw=0, pitch=0, roll=0):
		self.enabled = 1
		self.parent = None
		self.children = []
		self.userArgs = {}
		self.x = x
		self.y = y
		self.z = z
		self.yaw = yaw
		self.pitch = pitch
		self.roll = roll

	cpdef object getParent(self):
		return self.parent

	cpdef list getChildren(self):
		return self.children

	cpdef dict user(self):
		return self.userArgs

	cpdef double getX(self):
		return self.x

	cpdef void setX(self, double x):
		self.x = x

	cpdef void addX(self, double x):
		self.x += x

	cpdef double getY(self):
		return self.y

	cpdef void setY(self, double y):
		self.y = y

	cpdef void addY(self, double y):
		self.y += y

	cpdef double getZ(self):
		return self.z

	cpdef void setZ(self, double z):
		self.z = z

	cpdef void addZ(self, double z):
		self.z += z

	cpdef double getYaw(self):
		return self.yaw

	cpdef void setYaw(self, double y):
		self.yaw = y

	cpdef void addYaw(self, double y):
		self.yaw += y

	cpdef double getPitch(self):
		return self.pitch

	cpdef void setPitch(self, double p):
		self.pitch = p

	cpdef void addPitch(self, double p):
		self.pitch += p

	cpdef double getRoll(self):
		return self.roll

	cpdef void setRoll(self, double r):
		self.roll = r

	cpdef void addRoll(self, double r):
		self.roll = r

	cpdef (double, double, double) getPos(self):
		return (self.x, self.y, self.z)

	cpdef void setPos(self, (double, double, double) pos):
		self.x = pos[0]
		self.y = pos[1]
		self.z = pos[2]

	cpdef void addPos(self, (double, double, double) pos):
		self.x += pos[0]
		self.y += pos[1]
		self.z += pos[2]

	cpdef (double, double, double) getRotation(self):
		return (self.yaw, self.pitch, self.roll)

	cpdef void setRotation(self, (double, double, double) rot):
		self.yaw = rot[0]
		self.pitch = rot[1]
		self.roll = rot[2]

	cpdef void addRotation(self, (double, double, double) rot):
		self.yaw += rot[0]
		self.pitch += rot[1]
		self.roll += rot[2]

	cpdef void enable(self):
		self.enabled = 1

	cpdef void disable(self):
		self.enabled = 0

	cpdef void construct(self, vector[(double, double, double)] points, (unsigned char, unsigned char, unsigned char, unsigned char) color):
		self._polyVecX.clear()
		self._polyVecY.clear()
		self._polyVecZ.clear()
		for self._polyPoint in range(points.size()):
			self._polyVecX.push_back(points[self._polyPoint][0])
			self._polyVecY.push_back(points[self._polyPoint][1])
			self._polyVecZ.push_back(points[self._polyPoint][2])
		self.Xverts.push_back(self._polyVecX)
		self.Yverts.push_back(self._polyVecY)
		self.Zverts.push_back(self._polyVecZ)
		self.polyVertsColorR.push_back(color[0])
		self.polyVertsColorG.push_back(color[1])
		self.polyVertsColorB.push_back(color[2])
		self.polyVertsColorA.push_back(color[3])

	cpdef void reparent(self, Node newParent):
		if self.parent != None:
			self.parent.children.remove(self)
		newParent.children.append(self)
		newParent.children = list(set(newParent.children))
		self.parent = newParent

	cpdef void adopt(self, Node newChild):
		if newChild.parent != None:
			newChild.parent.children.remove(newChild)
		newChild.parent = self
		self.children.append(newChild)
		self.children = list(set(self.children))

	# Returns vec[X], vec[Y], vec[Z], r, g, b, a
	cpdef (vector[vector[double]], vector[vector[double]], vector[vector[double]], vector[unsigned char], vector[unsigned char], vector[unsigned char], vector[unsigned char]) getHierarchy(self):
		self._res[0].clear()
		self._res[1].clear()
		self._res[2].clear()
		self._res[3].clear()
		self._res[4].clear()
		self._res[5].clear()
		self._res[6].clear()
		if not self.enabled:
			return self._res
		self._res[0].insert(self._res[0].end(), self.Xverts.begin(), self.Xverts.end())
		self._res[1].insert(self._res[1].end(), self.Yverts.begin(), self.Yverts.end())
		self._res[2].insert(self._res[2].end(), self.Zverts.begin(), self.Zverts.end())
		self._res[3].insert(self._res[3].end(), self.polyVertsColorR.begin(), self.polyVertsColorR.end())
		self._res[4].insert(self._res[4].end(), self.polyVertsColorG.begin(), self.polyVertsColorG.end())
		self._res[5].insert(self._res[5].end(), self.polyVertsColorB.begin(), self.polyVertsColorB.end())
		self._res[6].insert(self._res[6].end(), self.polyVertsColorA.begin(), self.polyVertsColorA.end())
		for self._childIter in range(len(self.children)):
			self._childHie = self.children[self._childIter].getHierarchy()
			# Why didn't i use a for loop? Because it will think that it's accessed using Python and use list rather than vector
			self._res[0].insert(self._res[0].end(), self._childHie[0].begin(), self._childHie[0].end())
			self._res[1].insert(self._res[1].end(), self._childHie[1].begin(), self._childHie[1].end())
			self._res[2].insert(self._res[2].end(), self._childHie[2].begin(), self._childHie[2].end())
			self._res[3].insert(self._res[3].end(), self._childHie[3].begin(), self._childHie[3].end())
			self._res[4].insert(self._res[4].end(), self._childHie[4].begin(), self._childHie[4].end())
			self._res[5].insert(self._res[5].end(), self._childHie[5].begin(), self._childHie[5].end())
			self._res[6].insert(self._res[6].end(), self._childHie[6].begin(), self._childHie[6].end())
		self._sX = dsin(self.yaw)
		self._sY = dsin(self.pitch)
		self._sZ = dsin(self.roll)
		self._cX = dcos(self.yaw)
		self._cY = dcos(self.pitch)
		self._cZ = dcos(self.roll)
		for self._polyIter in range(self._res[0].size()):
			for self._pointIter in range(self._res[0][self._polyIter].size()):
				self._vec3 = rotateVec3tri((self._res[0][self._polyIter][self._pointIter],
					self._res[1][self._polyIter][self._pointIter],
					self._res[2][self._polyIter][self._pointIter]),
					self._sX, self._cX, self._sY, self._cY, self._sZ, self._cZ)
				self._res[0][self._polyIter][self._pointIter] = self._vec3[0] + self.x
				self._res[1][self._polyIter][self._pointIter] = self._vec3[1] + self.y
				self._res[2][self._polyIter][self._pointIter] = self._vec3[2] + self.z
		return self._res


cdef class Renderer:
	cdef double x, y, z, yaw, pitch, roll, fov, _fov, clipD
	cdef unsigned int width, height, _offset
	cdef (unsigned char, unsigned char, unsigned char) bg
	cpdef object surf
	cpdef Node root
	cdef (vector[vector[double]], vector[vector[double]], vector[vector[double]], vector[unsigned char], vector[unsigned char], vector[unsigned char], vector[unsigned char]) _hie

	def __init__(self, surf, Node rootNode, (unsigned char, unsigned char, unsigned char) background, double clipDistance = 0.05):
		if type(surf) != pygame.Surface:
			raise Exception('Renderer3D surface parameter isn\'t a pygame.Surface!')
		self.root = rootNode
		self.clipD = clipDistance
		self.surf = surf
		self.bg = background
		self.width = surf.get_width()
		self.height = surf.get_height()
		self.x = self.y = self.z = self.yaw = self.pitch = self.roll = 0
		self.fov = 90
		if self.height > self.width:
			self._offset = self.height
		else:
			self._offset = self.width
		self._fov = self._offset/2

	cpdef double getX(self):
		return self.x

	cpdef void setX(self, double x):
		self.x = x

	cpdef void addX(self, double x):
		self.x += x

	cpdef double getY(self):
		return self.y

	cpdef void setY(self, double y):
		self.y = y

	cpdef void addY(self, double y):
		self.y += y

	cpdef double getZ(self):
		return self.z

	cpdef void setZ(self, double z):
		self.z = z

	cpdef void addZ(self, double z):
		self.z += z

	cpdef double getYaw(self):
		return self.yaw

	cpdef void setYaw(self, double y):
		self.yaw = y

	cpdef void addYaw(self, double y):
		self.yaw += y

	cpdef double getPitch(self):
		return self.pitch

	cpdef void setPitch(self, double p):
		self.pitch = p

	cpdef void addPitch(self, double p):
		self.pitch += p

	cpdef double getRoll(self):
		return self.roll

	cpdef void setRoll(self, double r):
		self.roll = r

	cpdef void addRoll(self, double r):
		self.roll = r

	cpdef (double, double, double) getPos(self):
		return (self.x, self.y, self.z)

	cpdef void setPos(self, (double, double, double) pos):
		self.x = pos[0]
		self.y = pos[1]
		self.z = pos[2]

	cpdef void addPos(self, (double, double, double) pos):
		self.x += pos[0]
		self.y += pos[1]
		self.z += pos[2]

	cpdef (double, double, double) getRotation(self):
		return (self.yaw, self.pitch, self.roll)

	cpdef void setRotation(self, (double, double, double) rot):
		self.yaw = rot[0]
		self.pitch = rot[1]
		self.roll = rot[2]

	cpdef void addRotation(self, (double, double, double) rot):
		self.yaw += rot[0]
		self.pitch += rot[1]
		self.roll += rot[2]

	cpdef double getFOV(self):
		return self.fov

	cpdef void setFOV(self, double fov):
		self.fov = fov
		self._fov = dtan(fov/2)/2*self._offset

	cpdef void addFOV(self, double fov):
		self.fov += fov
		self._fov = dtan(self.fov/2)/2*self._offset

	cdef vector[(double, double, double)] VEC
	cdef unsigned int _polyIter, _pointIter
	cpdef void render(self, unsigned char blur=255, unsigned int placeholder = 0):
		self.surf.fill((self.bg[0], self.bg[1], self.bg[2], blur))
		self.root.setPos((-self.x, -self.y, -self.z))
		self.root.setRotation((-self.yaw, -self.pitch, -self.roll))
		self._hie = self.root.getHierarchy()
		for self._polyIter in range(self._hie[0].size()):
			self.VEC.clear()
			for self._pointIter in range(self._hie[0][self._polyIter].size()):
				self.VEC.push_back((self._hie[0][self._polyIter][self._pointIter],
					self._hie[1][self._polyIter][self._pointIter],
					self._hie[2][self._polyIter][self._pointIter]))
			self.VEC = clipPolygonBehind(self.VEC)
			for placeholder in range(self.VEC.size()):
				pass
