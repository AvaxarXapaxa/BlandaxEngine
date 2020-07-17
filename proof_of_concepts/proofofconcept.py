import math
# No radian units!!!
sin = lambda x: math.sin(math.radians(x))
cos = lambda x: math.cos(math.radians(x))
tan = lambda x: math.tan(math.radians(x))
map = lambda val, xmin, xmax, ymin, ymax: (val - xmin) * (ymax - ymin) / (xmax - xmin) + ymin

def translate3d(x, y, z, camRotX, camRotY, camRotZ):
	# RotX: Yaw, horizontal rotate. 0 degree facing Z+
	# RotY: Pitch, vertical rotate. Straight
	# RotZ: Roll, rotate 2D
	oldX = x
	oldY = y
	if not ((camRotX % 360 == 0) and (camRotY % 360 == 0)):
		# Do some rotation matrix witchcraft
		sX = sin(camRotX)
		sY = sin(camRotY)
		cX = cos(camRotX)
		cY = cos(camRotY)
		x = x * cX + z * sX
		z = z * cX - oldX * sX
		y = y * cY + z * sY
		z = z * cY - oldY * sY
		oldX = x
		oldY = y
	if not camRotZ % 360 == 0:
		camRotZ *= -1
		sZ = sin(camRotZ)
		cZ = cos(camRotZ)
		x = oldX * cZ - oldY * sZ
		y = oldX * sZ + oldY * cZ
	return x, y, z

def clipLine(x1, y1, x2, y2):
	if -1 <= x1 <= 1 and -1 <= y1 <= 1 and \
	-1 <= x2 <= 1 and -1 <= y2 <= 1:
		# Both points are inside
		return (x1, y1), (x2, y2)
	if (x1 < -1 and x2 < -1) or (x1 > 1 and x2 > 1) or \
	(y1 < -1 and y2 < -1) or (y1 > 1 and y2 > 1):
		# Both points are outside and don't intersect
		return
	if y1 == y2:
		# Horizontal line for clipping
		if x1 < -1:
			x1 = -1
		if x1 > 1:
			x1 = 1
		if x2 < -1:
			x2 = -1
		if x2 > 1:
			x2 = 1
		return (x1, y1), (x2, y2)
	if x1 == x2:
		# Vertical line for clipping
		if y1 < -1:
			y1 = -1
		if y1 > 1:
			y1 = 1
		if y2 < -1:
			y2 = -1
		if y2 > 1:
			y2 = 1
		return (x1, y1), (x2, y2)

	# Constants
	bottom = 1
	top = 2
	left = 4
	right = 8

	# For this algorithm
	def cohenAlg(x, y):
		code = 0
		if y > 1:
			code |= top
		elif y < -1:
			code |= bottom
		if x < -1:
			code |= left
		elif x > 1:
			code |= right
		return code

	coh1 = cohenAlg(x1, y1)
	coh2 = cohenAlg(x2, y2)
	while coh1 or coh2:
		if coh1 & coh2:
			return
		if coh1:
			if coh1 & top:
				y = 1
				x = x1 + ((x2 - x1) * (y - y1)) / (y2 - y1)
			elif coh1 & bottom:
				y = -1
				x = x1 + ((x2 - x1) * (y - y1)) / (y2 - y1)
			elif coh1 & left:
				x = -1
				y = y1 + ((y2 - y1) * (x - x1)) / (x2 - x1);
			elif coh1 & right:
				x = 1
				y = y1 + ((y2 - y1) * (x - x1)) / (x2 - x1);
			x1, y1 = x, y
			coh1 = cohenAlg(x, y)
		else:
			if coh2 & top:
				y = 1
				x = x1 + ((x2 - x1) * (y - y1)) / (y2 - y1)
			elif coh2 & bottom:
				y = -1
				x = x1 + ((x2 - x1) * (y - y1)) / (y2 - y1)
			elif coh2 & left:
				x = -1
				y = y1 + ((y2 - y1) * (x - x1)) / (x2 - x1);
			elif coh2 & right:
				x = 1
				y = y1 + ((y2 - y1) * (x - x1)) / (x2 - x1);
			x2, y2 = x, y
			coh2 = cohenAlg(x, y)
	return (x1, y1), (x2, y2)

def clipBehind(x1, y1, z1, x2, y2, z2):
	# Clip line below 0 at Z axis
	if z1 >= 0 and z2 >= 0:
		return (x1, y1, z1), (x2, y2, z2)
	if z1 < 0 and z2 < 0:
		return
	if z1 == z2:
		if z1 < 0:
			z1 = z2 = 0
		return (x1, y1, z1), (x2, y2, z2)
	if z1 < 0 and z2 >= 0:
		return (map(0, z1, z2, x1, x2), map(0, z1, z2, y1, y2), 0.0), (x2, y2, z2)
	if z1 >= 0 and z2 < 0:
		return (x1, y1, z1), (map(0, z1, z2, x1, x2), map(0, z1, z2, y1, y2), 0.0)
	return

while True:
	# print('3D rotation program')
	# x, y, z = translate3d(float(input('X: ')), float(input('Y: ')), float(input('Z: ')), float(input('RotX: ')), float(input('RotY: ')), float(input('RotZ: ')))
	# print(f'Result: X: {x}; Y: {y}; Z: {z}')
	print(clipBehind(float(input('X1: ')), float(input('Y1: ')), float(input('Z1: ')), float(input('X2: ')), float(input('Y2: ')), float(input('Z2: '))))
