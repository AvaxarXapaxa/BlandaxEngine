# C libraries
from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp.map cimport map
from libcpp.algorithm cimport sort as csort
from libc.string cimport memcpy
from libc.math cimport sin as rsin, cos as rcos, tan as rtan,\
  asin as rasin, acos as racos, atan as ratan, atan2 as ratan2, pow, sqrt, cbrt

# Builtin Python modules
import sys, os
from sys import argv
from os import system as cmd, name as osname, environ as envvar,\
  chdir, getcwd as getdir, listdir, mkdir, rmdir, rename, remove
from os.path import isdir, isfile, islink, ismount, isabs, exists
import time, socket
from time import time, sleep
from _thread import start_new as thread
from sys import exit

# Other Python modules
envvar['PYGAME_HIDE_SUPPORT_PROMPT'] = 'hide'
import pygame, numpy
cimport numpy
del envvar['PYGAME_HIDE_SUPPORT_PROMPT']
pygame.init()

# Maths
cdef:
  # Constants
  double pi = 3.14159265358979323846
  double euler = 2.71828182845904523536

  # Functions
  double remap(double x, double amin, double amax, double bmin, double bmax):
    if (amax - amin) + bmin == 0:
      return bmin
    else:
      return (x - amin) * (bmax - bmin) / (amax - amin) + bmin

  double dot((double, double, double) x, (double, double, double) y):
    return x[0]*y[0]+x[1]*y[1]+x[2]*y[2]

  (double, double, double) cross((double, double, double) a, (double, double, double) b):
    return (a[1]*b[2] - a[2]*b[1],
      a[2]*b[0] - a[0]*b[2],
      a[0]*b[1] - a[1]*b[0])

  # No radians! Trigonometric functions
  double sin(double x):
    x = x % 360
    if x == 0 or x == 180:
      return 0
    elif x == 90:
      return 1
    elif x == 270:
      return -1
    else:
      return rsin(x/180*pi)

  double cos(double x):
    x = x % 360
    if x == 90 or x == 270:
      return 0
    elif x == 0:
      return 1
    elif x == 180:
      return -1
    else:
      return rcos(x/180*pi)

  double tan(double x):
    x = x % 180
    if x == 0:
      return 0
    elif x == 45:
      return 1
    elif x == 135:
      return -1
    else:
      return rtan(x/180*pi)

  double asin(double x):
    return rasin(x)/180*pi

  double acos(double x):
    return racos(x)/180*pi

  double atan(double x):
    return ratan(x)/180*pi

  double atan2(double x, double y):
    return ratan2(x, y)/180*pi


# Rotate 3D
# Rotate vector 3 according to origin (0, 0, 0)
# RotX: Yaw, horizontal rotate. 0 degree facing Z+
# RotY: Pitch, vertical rotate. Straight
# RotZ: Roll, rotate 2D
cdef (double, double, double) rotateVec3((double, double, double) vec3, double sX, double cX, double sY, double cY, double sZ, double cZ):
  cdef:
    double x = vec3[0]
    double y = vec3[1]
    double z = vec3[2]
    double oldX = x
    double oldY = y
  if not ((sX == 0) and (sY == 0)):
    x = x * cX + z * sX
    z = z * cX - oldX * sX
    y = y * cY + z * sY
    z = z * cY - oldY * sY
    oldX = x
    oldY = y
    sZ *= -1
    cZ *= -1
  if sZ != 0:
    x = oldX * cZ - oldY * sZ
    y = oldX * sZ + oldY * cZ
  return (x, y, z)

# Rotate 2D
cdef (double, double) rotateVec2((double, double) vec2, double sZ, double cZ):
  return (vec2[0] * cZ - vec2[1] * -sZ, vec2[0] * -sZ + vec2[1] * cZ)

# Line clipper 3D, Clip line behind Z 0
cdef ((double, double, double), (double, double, double), unsigned short int) clipLineBehind((double, double, double) pos1, (double, double, double) pos2):
  cdef:
    double x1 = pos1[0]
    double y1 = pos1[1]
    double z1 = pos1[2]
    double x2 = pos2[0]
    double y2 = pos2[1]
    double z2 = pos2[2]
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

# Polygon clipper 3D, Clip polygon behind Z 0
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
  cdef vector[(double, double, double)] retDupeFix # There's a chance that some point, coordinates
  cdef unsigned int ind # are duplicated. this eliminates duplicates
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

# Get the closest distance from a triangle
cdef (double, double, double) getClosestTriDistance((double, double, double) a, (double, double, double) b, (double, double, double) c):
  cdef (double, double, double) n = cross((b[0]-a[0], b[1]-a[1], b[2]-a[2]), (c[0]-a[0], c[1]-a[1], c[2]-a[2]))
  cdef double sq = sqrt(n[0]**2 + n[1]**2 + n[2]**2)
  if sq == 0:
    return (0, 0, 0)
  cdef (double, double, double) k = (1/sq*n[0], 1/sq*n[1], 1/sq*n[2])
  cdef double dotpro = dot(a, k)
  cdef (double, double, double) proj = (dotpro*k[0], dotpro*k[1], dotpro*k[2])
  return proj

# Checks if a triangle is clockwise
cdef char checkClockwise((double, double) a, (double, double) b, (double, double) c):
  return (C[1]-A[1]) * (B[0]-A[0]) < (B[1]-A[1]) * (C[0]-A[0])

# Checks if a point is inside a triangle
cdef char checkPointInTriangle((double, double) point, (double, double) a, (double, double) b, (double, double) c):
  cdef:
    double d1 = (point[0] - b[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (point[1] - b[1])
    double d2 = (point[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (point[1] - c[1])
    double d3 = (point[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (point[1] - a[1])
  return not (((d1 > 0) or (d2 > 0) or (d3 > 0)) and ((d1 < 0) or (d2 < 0) or (d3 < 0)))

# Check if 2 lines intersect
cpdef char checkLineIntersection(((double, double), (double, double)) l1, ((double, double), (double, double)) l2):
  return (((not checkClockwise(l1[0], l2[0], l2[1])) != (not checkClockwise(l1[1], l2[0], l2[1]))) and ((not checkClockwise(l1[0], l1[1], l2[0])) != (not checkClockwise(l1[0], l1[1], l2[1]))))
