#-! WARNING !-#
# This code may contain, dirty code, bizarre syntaxes,
# Excessive StackOverflow plagiarism/copy-pasting mostly,
# And, dirty loopholes of escaping Cython's parsing bugginess
# Read/Modify/Fork at your own risk...

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
import time, socket, json
from time import time, sleep
from json import dumps as make_json, loads as load_json
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

  double sign(double x):
    if x == 0:
      return 0
    elif x < 0:
      return -1
    elif x > 0:
      return 1

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
    elif x == 90:
      return 0
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
  x = x * cX + z * sX
  z = z * cX - oldX * sX
  y = y * cY + z * sY
  z = z * cY - oldY * sY
  oldX = x
  oldY = y
  x = oldX * cZ - oldY * -sZ
  y = oldX * -sZ + oldY * cZ
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
  cdef unsigned int point = 0
  cdef (double, double, double) prevPoint = (0, 0, 0)
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
  return (c[1]-a[1]) * (b[0]-a[0]) < (b[1]-a[1]) * (c[0]-a[0])

# Checks if a point is inside a triangle
cdef char checkPointInTriangle((double, double) point, (double, double) a, (double, double) b, (double, double) c):
  cdef:
    double d1 = (point[0] - b[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (point[1] - b[1])
    double d2 = (point[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (point[1] - c[1])
    double d3 = (point[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (point[1] - a[1])
  return not (((d1 > 0) or (d2 > 0) or (d3 > 0)) and ((d1 < 0) or (d2 < 0) or (d3 < 0)))

# Checks if a point is inside a polygon
cdef char checkPointInPolygon((double, double) point, vector[(double, double)] polygon):
  cdef:
    char c = 0
    unsigned int i = 0
    unsigned int j = polygon.size()-1
  while i < polygon.size():
    if (((polygon[i][1]>point[1]) != (polygon[j][1]>point[1])) and\
      (point[0] < (polygon[j][0]-polygon[i][0]) * (point[1]-polygon[i][1]) / (polygon[j][1]-polygon[i][1]) + polygon[i][0])):
      c = not c
    j = i
    i += 1
  return c

cdef double signed_tetra_volume((double, double, double) a, (double, double, double) b, (double, double, double) c, (double, double, double) d):
  return sign(dot(cross((b[0]-a[0], b[1]-a[1], b[2]-a[2]), (c[0]-a[0], c[1]-a[1], c[2]-a[2])), (d[0]-a[0], d[1]-a[1], d[2]-a[2]))/6)

# Give intersection point of a line and a triangle 3D plane
cdef ((double, double, double), char) checkLineTrianglePlaneIntersection((double, double, double) q1 , (double, double, double) q2, (double, double, double) p1, (double, double, double) p2, (double, double, double) p3):
  cdef:
    double s1 = signed_tetra_volume(q1,p1,p2,p3)
    double s2 = signed_tetra_volume(q2,p1,p2,p3)
    double s3 = signed_tetra_volume(q1,q2,p1,p2)
    double s4 = signed_tetra_volume(q1,q2,p2,p3)
    double s5 = signed_tetra_volume(q1,q2,p3,p1)
    (double, double, double) n = cross((p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]), (p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]))
    double t = dot((p1[0]-q1[0], p1[1]-q1[1], p1[2]-q1[2]),n) / dot((q2[0]-q1[0], q2[1]-q1[1], q2[2]-q1[2]),n)
  if s1 != s2 and s3 == s4 and s4 == s5:
    return (((q2[0]-q1[0])*t+q1[0], (q2[1]-q1[1])*t+q1[1], (q2[2]-q1[2])*t+q1[2]), 0)
  return ((0, 0, 0), 1) # Does not intersect

# Check if 2 lines intersect
cdef char checkLineIntersection(((double, double), (double, double)) l1, ((double, double), (double, double)) l2):
  return (((not checkClockwise(l1[0], l2[0], l2[1])) != (not checkClockwise(l1[1], l2[0], l2[1]))) and ((not checkClockwise(l1[0], l1[1], l2[0])) != (not checkClockwise(l1[0], l1[1], l2[1]))))

# Get FOV multiplier
cdef double getFOVmultiplier(double angle):
  return tan(angle/2)/2

# World object types
cdef struct Point_t:
  double x, y, z

cdef Point_t Pointf((double, double, double) vec3):
  cdef Point_t r
  r.x = vec3[0]
  r.y = vec3[1]
  r.z = vec3[2]
  return r

cdef struct Color_t:
  unsigned char r, g, b, a

cdef struct ImageBuffer_t:
  string buffer
  unsigned int width, height

cdef struct Triangle_t:
  Point_t a, b, c
  Color_t color
  char fullside

cdef struct Mesh_t:
  vector[Triangle_t] tris

cdef struct Prefab_t:
  Mesh_t mesh

cdef struct Instance_t:
  char enabled
  unsigned long int id
  Prefab_t prefab
  Point_t position
  Point_t rotation
  Point_t scale

cdef struct World_t:
  vector[Prefab_t] prefabs
  vector[Instance_t] instances

cdef struct Layer_t:
  char enabled
  World_t* world
  Point_t cameraPosition
  Point_t cameraRotation
  double fov
  unsigned char bgRed, bgGreen, bgBlue, bgAlpha

cdef object renderLayer(Layer_t layer, unsigned int width, unsigned int height):
  # Scaling
  cdef vector[Instance_t] scaledInstances
  cdef Instance_t scaledInstance
  cdef unsigned int sI, sT
  cdef unsigned int scaledInstancesLength = layer.world[0].instances.size()
  cdef unsigned int scaledTrianglesLength
  #print('scaledInstancesLength:', scaledInstancesLength)
  sI = 0
  while sI < scaledInstancesLength:
    #print('sI: ', sI)
    if not layer.world[0].instances[sI].enabled:
      #print(not layer.world[0].instances[sI].enabled)
      #print('Not enabled!')
      sI += 1
      continue
    scaledInstance = layer.world[0].instances[sI]
    sT = 0
    scaledTrianglesLength = scaledInstance.prefab.mesh.tris.size()
    #print('scaledTrianglesLength', scaledTrianglesLength)
    while sT < scaledTrianglesLength:
      #print('sT:', sT)
      scaledInstance.prefab.mesh.tris[sT].a.x *= scaledInstance.scale.x
      scaledInstance.prefab.mesh.tris[sT].a.y *= scaledInstance.scale.y
      scaledInstance.prefab.mesh.tris[sT].a.z *= scaledInstance.scale.z
      scaledInstance.prefab.mesh.tris[sT].b.x *= scaledInstance.scale.x
      scaledInstance.prefab.mesh.tris[sT].b.y *= scaledInstance.scale.y
      scaledInstance.prefab.mesh.tris[sT].b.z *= scaledInstance.scale.z
      scaledInstance.prefab.mesh.tris[sT].c.x *= scaledInstance.scale.x
      scaledInstance.prefab.mesh.tris[sT].c.y *= scaledInstance.scale.y
      scaledInstance.prefab.mesh.tris[sT].c.z *= scaledInstance.scale.z
      sT += 1
    #print(scaledInstance)
    scaledInstances.push_back(scaledInstance)
    sI += 1

  # Rotating
  cdef vector[Instance_t] rotatedInstances
  cdef Instance_t rotatedInstance
  cdef Point_t rotatedPoint
  cdef (double, double, double) rotated3vec
  cdef double sX, sY, sZ, cX, cY, cZ
  cdef unsigned int instancesLength = scaledInstances.size()
  cdef unsigned int triangleAmount
  #print('instancesLength', instancesLength)
  sI = 0
  while sI < instancesLength:
    #print('sI:', sI)
    rotatedInstance = scaledInstances[sI]
    sT = 0
    sX = sin(rotatedInstance.rotation.x)
    cX = cos(rotatedInstance.rotation.x)
    sY = sin(rotatedInstance.rotation.y)
    cY = cos(rotatedInstance.rotation.y)
    sZ = sin(rotatedInstance.rotation.z)
    cZ = cos(rotatedInstance.rotation.z)
    triangleAmount = rotatedInstance.prefab.mesh.tris.size()
    while sT < triangleAmount:
      #print('sT', sT)
      rotated3vec = rotateVec3((rotatedInstance.prefab.mesh.tris[sT].a.x, \
        rotatedInstance.prefab.mesh.tris[sT].a.y, \
        rotatedInstance.prefab.mesh.tris[sT].a.z), sX, cX, sY, cY, sZ, cZ)
      rotatedPoint = Pointf(rotated3vec)
      rotatedInstance.prefab.mesh.tris[sT].a = rotatedPoint
      rotated3vec = rotateVec3((rotatedInstance.prefab.mesh.tris[sT].b.x, \
        rotatedInstance.prefab.mesh.tris[sT].b.y, \
        rotatedInstance.prefab.mesh.tris[sT].b.z), sX, cX, sY, cY, sZ, cZ)
      rotatedPoint = Pointf(rotated3vec)
      rotatedInstance.prefab.mesh.tris[sT].b = rotatedPoint
      rotated3vec = rotateVec3((rotatedInstance.prefab.mesh.tris[sT].c.x, \
        rotatedInstance.prefab.mesh.tris[sT].c.y, \
        rotatedInstance.prefab.mesh.tris[sT].c.z), sX, cX, sY, cY, sZ, cZ)
      rotatedPoint = Pointf(rotated3vec)
      rotatedInstance.prefab.mesh.tris[sT].c = rotatedPoint
      sT += 1
    rotatedInstances.push_back(rotatedInstance)
    sI += 1

  # Collect the triangles and get the relative coordinates from the camera
  cdef vector[Triangle_t] triangles
  cdef Triangle_t triangle

  sI = 0
  while sI < instancesLength:
    sT = 0
    triangleAmount = rotatedInstances[sI].prefab.mesh.tris.size()
    while sT < triangleAmount:
      triangle = rotatedInstances[sI].prefab.mesh.tris[sT]
      triangle.a.x = (triangle.a.x + rotatedInstances[sI].position.x) - layer.cameraPosition.x
      triangle.a.y = (triangle.a.y + rotatedInstances[sI].position.y) - layer.cameraPosition.y
      triangle.a.z = (triangle.a.z + rotatedInstances[sI].position.z) - layer.cameraPosition.z
      triangle.b.x = (triangle.b.x + rotatedInstances[sI].position.x) - layer.cameraPosition.x
      triangle.b.y = (triangle.b.y + rotatedInstances[sI].position.y) - layer.cameraPosition.y
      triangle.b.z = (triangle.b.z + rotatedInstances[sI].position.z) - layer.cameraPosition.z
      triangle.c.x = (triangle.c.x + rotatedInstances[sI].position.x) - layer.cameraPosition.x
      triangle.c.y = (triangle.c.y + rotatedInstances[sI].position.y) - layer.cameraPosition.y
      triangle.c.z = (triangle.c.z + rotatedInstances[sI].position.z) - layer.cameraPosition.z
      triangles.push_back(triangle)
      sT += 1
    sI += 1

  # Clip side 1 (TOP)
  cdef vector[Triangle_t] clippedTriangles
  cdef Triangle_t clippedTriangle
  cdef vector[(double, double, double)] clips
  triangleAmount = triangles.size()

  cdef double csX = sin(-layer.cameraRotation.x)
  cdef double ccX = cos(-layer.cameraRotation.x)
  cdef double csY = sin(-layer.cameraRotation.y-layer.fov/2)
  cdef double ccY = cos(-layer.cameraRotation.y-layer.fov/2)
  cdef double csZ = sin(-layer.cameraRotation.z)
  cdef double ccZ = cos(-layer.cameraRotation.z)

  sT = 0
  while sT < triangleAmount:
    clips.clear()
    triangle = triangles[sT]
    triangles[sT].a = Pointf(rotateVec3((triangle.a.x, triangle.a.y, triangle.a.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].b = Pointf(rotateVec3((triangle.b.x, triangle.b.y, triangle.b.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].c = Pointf(rotateVec3((triangle.c.x, triangle.c.y, triangle.c.z), csX, ccX, csY, ccY, csZ, ccZ))
    clips.push_back((triangles[sT].a.x, triangles[sT].a.y, triangles[sT].a.z))
    clips.push_back((triangles[sT].b.x, triangles[sT].b.y, triangles[sT].b.z))
    clips.push_back((triangles[sT].c.x, triangles[sT].c.y, triangles[sT].c.z))
    #print('clips before', clips)
    clips = clipPolygonBehind(clips)
    #print('clips: ', clips)
    if clips.size() == 3:
      clippedTriangle = triangles[sT]
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[1])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[2])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
    elif clips.size() == 4:
      clippedTriangle = triangles[sT]
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[1])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[2])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[2])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[3])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
    sT += 1

  #print('clipped result1', clippedTriangles)
  # Clip side 2 bottom
  triangles = clippedTriangles
  clippedTriangles.clear()
  triangleAmount = triangles.size()

  csX = sin(0)
  ccX = cos(0)
  csY = sin(2*(layer.fov/2))
  ccY = cos(2*(layer.fov/2))
  csZ = sin(0)
  ccZ = cos(0)

  sT = 0
  while sT < triangleAmount:
    clips.clear()
    triangle = triangles[sT]
    triangles[sT].a = Pointf(rotateVec3((triangle.a.x, triangle.a.y, triangle.a.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].b = Pointf(rotateVec3((triangle.b.x, triangle.b.y, triangle.b.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].c = Pointf(rotateVec3((triangle.c.x, triangle.c.y, triangle.c.z), csX, ccX, csY, ccY, csZ, ccZ))
    clips.push_back((triangles[sT].a.x, triangles[sT].a.y, triangles[sT].a.z))
    clips.push_back((triangles[sT].b.x, triangles[sT].b.y, triangles[sT].b.z))
    clips.push_back((triangles[sT].c.x, triangles[sT].c.y, triangles[sT].c.z))
    #print('clips before', clips)
    clips = clipPolygonBehind(clips)
    #print('clips: ', clips)
    if clips.size() == 3:
      clippedTriangle = triangles[sT]
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[1])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[2])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
    elif clips.size() == 4:
      clippedTriangle = triangles[sT]
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[1])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[2])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[2])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[3])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
    sT += 1

  triangles = clippedTriangles
  triangleAmount = triangles.size()

  csX = sin(0)
  ccX = cos(0)
  csY = sin(-layer.fov/2)
  ccY = cos(-layer.fov/2)
  csZ = sin(0)
  ccZ = cos(0)

  sT = 0
  while sT < triangleAmount:
    triangle = triangles[sT]
    triangles[sT].a = Pointf(rotateVec3((triangle.a.x, triangle.a.y, triangle.a.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].b = Pointf(rotateVec3((triangle.b.x, triangle.b.y, triangle.b.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].c = Pointf(rotateVec3((triangle.c.x, triangle.c.y, triangle.c.z), csX, ccX, csY, ccY, csZ, ccZ))
    sT += 1
  clippedTriangles = triangles

  #print('clipped result2', clippedTriangles)
  # Clip side 3 left
  triangles = clippedTriangles
  clippedTriangles.clear()
  triangleAmount = triangles.size()

  csX = sin(-layer.fov/2)
  ccX = cos(-layer.fov/2)
  csY = sin(0)
  ccY = cos(0)
  csZ = sin(0)
  ccZ = cos(0)

  sT = 0
  while sT < triangleAmount:
    clips.clear()
    triangle = triangles[sT]
    triangles[sT].a = Pointf(rotateVec3((triangle.a.x, triangle.a.y, triangle.a.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].b = Pointf(rotateVec3((triangle.b.x, triangle.b.y, triangle.b.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].c = Pointf(rotateVec3((triangle.c.x, triangle.c.y, triangle.c.z), csX, ccX, csY, ccY, csZ, ccZ))
    clips.push_back((triangles[sT].a.x, triangles[sT].a.y, triangles[sT].a.z))
    clips.push_back((triangles[sT].b.x, triangles[sT].b.y, triangles[sT].b.z))
    clips.push_back((triangles[sT].c.x, triangles[sT].c.y, triangles[sT].c.z))
    #print('clips before', clips)
    clips = clipPolygonBehind(clips)
    #print('clips: ', clips)
    if clips.size() == 3:
      clippedTriangle = triangles[sT]
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[1])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[2])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
    elif clips.size() == 4:
      clippedTriangle = triangles[sT]
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[1])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[2])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[2])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[3])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
    sT += 1

  #print('clipped result3', clippedTriangles)
  # Clip side 4 right
  triangles = clippedTriangles
  clippedTriangles.clear()
  triangleAmount = triangles.size()

  csX = sin(2*(layer.fov/2))
  ccX = cos(2*(layer.fov/2))
  csY = sin(0)
  ccY = cos(0)
  csZ = sin(0)
  ccZ = cos(0)

  sT = 0
  while sT < triangleAmount:
    clips.clear()
    triangle = triangles[sT]
    triangles[sT].a = Pointf(rotateVec3((triangle.a.x, triangle.a.y, triangle.a.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].b = Pointf(rotateVec3((triangle.b.x, triangle.b.y, triangle.b.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].c = Pointf(rotateVec3((triangle.c.x, triangle.c.y, triangle.c.z), csX, ccX, csY, ccY, csZ, ccZ))
    clips.push_back((triangles[sT].a.x, triangles[sT].a.y, triangles[sT].a.z))
    clips.push_back((triangles[sT].b.x, triangles[sT].b.y, triangles[sT].b.z))
    clips.push_back((triangles[sT].c.x, triangles[sT].c.y, triangles[sT].c.z))
    #print('clips before', clips)
    clips = clipPolygonBehind(clips)
    #print('clips: ', clips)
    if clips.size() == 3:
      clippedTriangle = triangles[sT]
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[1])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[2])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
    elif clips.size() == 4:
      clippedTriangle = triangles[sT]
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[1])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[2])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
      clippedTriangle.a = Pointf(clips[0])
      if clippedTriangle.a.z == 0: clippedTriangle.a.z = 0.005
      clippedTriangle.b = Pointf(clips[2])
      if clippedTriangle.b.z == 0: clippedTriangle.b.z = 0.005
      clippedTriangle.c = Pointf(clips[3])
      if clippedTriangle.c.z == 0: clippedTriangle.c.z = 0.005
      clippedTriangles.push_back(clippedTriangle)
    sT += 1

  #print('clipped result4', clippedTriangles)

  # Make everything normal again
  triangles = clippedTriangles
  triangleAmount = triangles.size()

  csX = sin(-layer.fov/2)
  ccX = cos(-layer.fov/2)
  csY = sin(0)
  ccY = cos(0)
  csZ = sin(0)
  ccZ = cos(0)

  sT = 0
  while sT < triangleAmount:
    triangle = triangles[sT]
    triangles[sT].a = Pointf(rotateVec3((triangle.a.x, triangle.a.y, triangle.a.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].b = Pointf(rotateVec3((triangle.b.x, triangle.b.y, triangle.b.z), csX, ccX, csY, ccY, csZ, ccZ))
    triangles[sT].c = Pointf(rotateVec3((triangle.c.x, triangle.c.y, triangle.c.z), csX, ccX, csY, ccY, csZ, ccZ))
    sT += 1
  clippedTriangles = triangles

  #print('clipped result', clippedTriangles)
  cdef vector[Triangle_t] orderedTriangles
  cdef map[double, Triangle_t] distanceMap
  cdef vector[double] distances
  cdef double distance, dOffset, originalDistance
  sT = 0
  triangleAmount = clippedTriangles.size()
  #print('triangleAmount', triangleAmount)
  while sT < triangleAmount:
    dOffset = 0.001
    distance = clippedTriangles[sT].a.z
    if distance < clippedTriangles[sT].b.z:
      distance = clippedTriangles[sT].b.z
    if distance < clippedTriangles[sT].c.z:
      distance = clippedTriangles[sT].c.z
    originalDistance = distance
    while distanceMap.count(distance) > 0:
      distance = originalDistance + dOffset
      dOffset /= 1.25
    distanceMap[distance] = clippedTriangles[sT]
    distances.push_back(distance)
    sT += 1
  #print('Distances;', distanceMap)
  csort(distances.begin(), distances.end())
  #print('Sorted Distances:', distances)
  sT = distances.size()-1
  cdef Triangle_t renderedTriangle
  cdef int x1, y1, x2, y2, x3, y3, fov
  if width > height:
    fov = int(getFOVmultiplier(layer.fov)*width)
  else:
    fov = int(getFOVmultiplier(layer.fov)*height)

  #print('fov is', fov)
  cdef object surf
  surf = pygame.Surface((width, height)).convert_alpha()
  surf.fill((layer.bgRed, layer.bgGreen, layer.bgBlue, layer.bgAlpha))

  while sT != (1<<32)-1: # sT is an unsigned int interator
    #print('sT', sT)
    renderedTriangle = distanceMap[distances[sT]]
    #print('renderedTriangle', renderedTriangle)
    x1 = int(fov*(renderedTriangle.a.x/renderedTriangle.a.z))
    y1 = int(fov*(renderedTriangle.a.y/renderedTriangle.a.z))
    x2 = int(fov*(renderedTriangle.b.x/renderedTriangle.b.z))
    y2 = int(fov*(renderedTriangle.b.y/renderedTriangle.b.z))
    x3 = int(fov*(renderedTriangle.c.x/renderedTriangle.c.z))
    y3 = int(fov*(renderedTriangle.c.y/renderedTriangle.c.z))
    #print('Calculated!')
    #print('Rendering... ', (x1, y1), (x2, y2), (x3, y3))
    if checkClockwise((x1, y1), (x2, y2), (x3, y3)) or clippedTriangle.fullside:
      #print('Is clockwise! Rendering...')
      x1 = x1+(width//2)
      y1 = -y1+(height//2)
      x2 = x2+(width//2)
      y2 = -y2+(height//2)
      x3 = x3+(width//2)
      y3 = -y3+(height//2)
      #print('Drawing!!', (x1, y1), (x2, y2), (x3, y3))
      pygame.draw.polygon(surf, (renderedTriangle.color.r, renderedTriangle.color.g, renderedTriangle.color.b, renderedTriangle.color.a), \
        ((x1, y1), (x2, y2), (x3, y3)))
      #print('Finished drawing!')
    sT -= 1
  #print('Going home!')
  return surf

cdef Layer_t ll
cdef World_t w
cdef Instance_t a
cdef Triangle_t t
a.enabled = 1
a.position = Pointf((0, 0, 10))
t.a = Pointf((-1, -1, -1))
t.b = Pointf((-1, 1, -1))
t.c = Pointf((1, -1, -1))
t.fullside = 1
t.color.r = 255
t.color.g = 0
t.color.b = 0
t.color.a = 255
a.prefab.mesh.tris.push_back(t)
t.a = Pointf((1, 1, -1))
t.b = Pointf((-1, 1, -1))
t.c = Pointf((1, -1, -1))
t.fullside = 1
t.color.r = 0
t.color.g = 255
t.color.b = 0
t.color.a = 255
a.prefab.mesh.tris.push_back(t)

t.a = Pointf((-1, -1, 1))
t.b = Pointf((-1, 1, 1))
t.c = Pointf((1, -1, 1))
t.fullside = 1
t.color.r = 255
t.color.g = 255
t.color.b = 0
t.color.a = 255
a.prefab.mesh.tris.push_back(t)
t.a = Pointf((1, 1, 1))
t.b = Pointf((-1, 1, 1))
t.c = Pointf((1, -1, 1))
t.fullside = 1
t.color.r = 0
t.color.g = 0
t.color.b = 255
t.color.a = 255
a.prefab.mesh.tris.push_back(t)

t.a = Pointf((-1, 1, -1))
t.b = Pointf((-1, 1, 1))
t.c = Pointf((1, 1, -1))
t.fullside = 1
t.color.r = 255
t.color.g = 0
t.color.b = 255
t.color.a = 255
a.prefab.mesh.tris.push_back(t)
t.a = Pointf((1, 1, 1))
t.b = Pointf((-1, 1, 1))
t.c = Pointf((1, 1, -1))
t.fullside = 1
t.color.r = 0
t.color.g = 255
t.color.b = 255
t.color.a = 255
a.prefab.mesh.tris.push_back(t)

t.a = Pointf((-1, -1, -1))
t.b = Pointf((-1, -1, 1))
t.c = Pointf((1, -1, -1))
t.fullside = 1
t.color.r = 255
t.color.g = 255
t.color.b = 255
t.color.a = 255
a.prefab.mesh.tris.push_back(t)
t.a = Pointf((1, -1, 1))
t.b = Pointf((-1, -1, 1))
t.c = Pointf((1, -1, -1))
t.fullside = 1
t.color.r = 0
t.color.g = 0
t.color.b = 0
t.color.a = 255
a.prefab.mesh.tris.push_back(t)

a.scale = Pointf((1, 1, 1))
w.instances.push_back(a)
ll.world = &w
ll.cameraPosition = Pointf((0, 0, 0))
ll.cameraRotation = Pointf((0, 0, 0))
ll.fov = 90

win = pygame.display.set_mode((700, 700))
clock = pygame.time.Clock()
cdef object key

while True:
  for e in pygame.event.get():
    pass
  win.fill((128, 128, 128))
  #w.instances[0].rotation.x += 1
  #w.instances[0].rotation.y += 1
  #w.instances[0].rotation.z += 1
  print(ll.cameraPosition, ll.cameraRotation)
  key = pygame.key.get_pressed()
  if key[pygame.K_w]:
    ll.cameraPosition.z += 0.05
  if key[pygame.K_a]:
    ll.cameraPosition.x -= 0.05
  if key[pygame.K_s]:
    ll.cameraPosition.z -= 0.05
  if key[pygame.K_d]:
    ll.cameraPosition.x += 0.05
  if key[pygame.K_UP]:
    ll.cameraRotation.y += 1
  if key[pygame.K_DOWN]:
    ll.cameraRotation.y -= 1
  if key[pygame.K_LEFT]:
    ll.cameraRotation.x -= 1
  if key[pygame.K_RIGHT]:
    ll.cameraRotation.x += 1
  win.blit(renderLayer(ll, 700, 700), (0, 0))
  pygame.display.set_caption(f'Blandax Engine Test FPS: {clock.get_fps()}')
  pygame.display.flip()
  clock.tick(60)
