'''
Python EdgeBreaker algorithm working with the Open3D library.

Implemented and improved from:
https://www.cs.cmu.edu/~alla/edgebreaker_simple.pdf
'''

import cProfile
from os import read
import numpy
import open3d
import sys
import time

from dataclasses import dataclass
from datetime import datetime
from pstats import Stats, SortKey


# ------------------------------------------------------------
# Global variables
# ------------------------------------------------------------

## DEBUG AND VISUALIZATION

_doProfiling = False				# Do some profiling

_debug = False						# True if you want to enable color changes and delay between draw calls
_debugPrint = False					# True if you want to enable debug prints

_debugDelayPerFrame = 0.00			# Delay between draw calls

_debugTriangleColor = [240, 0, 0]	# The current triangle color for debug-drawing triangles (to use, divide by 255)
_debugColorOffset = 24				# For each triangle, 24 will be added or removed from one of the RGB component
_debugRGBIndex = 1					# RGB index, 0 = R, 1 = G, 2 = B
_debugRGBIncrease = True			# When true, we add _debugColorOffset for each triangle, else we subtract _debugColorOffset

_visualizer = None					# The visualizer (the window)
_lastUpdateTime = -1				# Last visual update time in seconds


## EDGEBREAKER RELATED

_heMesh = None 				# The mesh containing half-edges data

_clers = ""					# String storing the CLERS steps of the EdgeBreaker algorithm's path
_deltas = []				# List of 3D points/vectors storing the first points and the correction vectors
_normals = []				# TODO

_marked = []				# List of bool indicating whether a vertex has already been visited: M in the paper
_inDeltas = []				# List of bool indicating whether a vertex position has already been saved in _deltas array
_flagged = []				# List of bool indicating whether a triangle has already been visited: U in the paper

_startingHalfEdge = 71		# Select first half-edge to begin the EdgeBreaker algorithm
_previousHeId = -1			# Id of the previously visited half-edge, used to calculate delta vector(_previousHeId → halfEdgeId)


## DATA STORAGE

# Store frequently accessed data in numpy arrays to accelerate access time
_halfEdges = None
_vertices = None
_triangles = None


# ------------------------------------------------------------
# Data access functions
# ------------------------------------------------------------

## VERTICES

def getVertexId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return _halfEdges[halfEdgeId].vertex_indices[0]


def getNextVertexId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return getVertexId(getNextHeId(halfEdgeId))


def getPreviousVertexId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return getVertexId(getPreviousHeId(halfEdgeId))


def getOppositeVertexId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return getVertexId(getOppositeCornerHeId(halfEdgeId))


def getLeftVertexId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return getVertexId(getLeftCornerHeId(halfEdgeId))


def getRightVertexId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return getVertexId(getRightCornerHeId(halfEdgeId))


## HALF-EDGES

def getHalfEdge(halfEdgeId):
	return _halfEdges[halfEdgeId]


def getNextHeId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return _halfEdges[halfEdgeId].next


def getPreviousHeId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return getNextHeId(getNextHeId(halfEdgeId))


def getTwinHeId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return _halfEdges[halfEdgeId].twin


def getOppositeCornerHeId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return getPreviousHeId(getTwinHeId(getNextHeId(halfEdgeId)))


def getLeftCornerHeId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return getPreviousHeId(getTwinHeId(halfEdgeId))


def getRightCornerHeId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return getPreviousHeId(getTwinHeId(getPreviousHeId(halfEdgeId)))


## TRIANGLES

def getTriangleFromHeId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return _halfEdges[halfEdgeId].triangle_index


## DISTANCE VECTORS

def getDistanceVectorFromVerticesId(fromVertexId, toVertexId):
	return _vertices[toVertexId] - _vertices[fromVertexId]


def getDistanceVectorFromHeId(fromHeId, toHeId):
	fromVertexId = getVertexId(fromHeId)
	toVertexId = getVertexId(toHeId)
	return getDistanceVectorFromVerticesId(fromVertexId, toVertexId)


## VERTEX POSITION

def getVertexPosFromVertexId(vertexId):
	return _vertices[vertexId]


def getVertexPosFromHeId(halfEdgeId):
	return _vertices[getVertexId(halfEdgeId)]


# ------------------------------------------------------------
# FLAGS
# ------------------------------------------------------------

## MARKS

def mark(halfEdgeId):
	global _marked

	if halfEdgeId != -1:
		_marked[getVertexId(halfEdgeId)] = True


def isMarked(halfEdgeId):
	global _marked

	if halfEdgeId == -1:
		return True
	else:
		return _marked[getVertexId(halfEdgeId)]


## FLAGS

def flag(halfEdgeId):
	global _flagged

	if halfEdgeId != -1:
		_flagged[getTriangleFromHeId(halfEdgeId)] = True


def isFlagged(halfEdgeId):
	global _flagged
	
	if halfEdgeId == -1:
		return True
	else:
		return _flagged[getTriangleFromHeId(halfEdgeId)]


## DELTAS

def addPosToDeltas(halfEdgeId):
	global _deltas, _inDeltas

	vertexId = getVertexId(halfEdgeId)
	pos = getVertexPosFromHeId(halfEdgeId)
	_deltas.append(pos)
	_inDeltas[vertexId] = True
	# debugPrint(f'# Add pos {pos}')


def addVectorToDeltas(previousHalfEdgeId, halfEdgeId):
	global _deltas, _inDeltas

	vertexId = getVertexId(halfEdgeId)
	vector = getDistanceVectorFromHeId(previousHalfEdgeId, halfEdgeId)
	_deltas.append(vector)
	_inDeltas[vertexId] = True
	# debugPrint(f'# Add vector {vector}')


# ------------------------------------------------------------
# Debug functions
# ------------------------------------------------------------

def debugInit():
	global _debug, _visualizer

	if _debug:
		_visualizer = open3d.visualization.Visualizer()
		_visualizer.create_window()
		_visualizer.add_geometry(_heMesh)

		_heMesh.paint_uniform_color([0.6, 0.6, 0.6])


def debugEnd():
	global _debug

	if _debug:
		_debug = False
		_visualizer.run()


def debugPrint(string):
	if _debugPrint:
		print(string)
	

def debugDrawAndWait():
	global _lastUpdateTime

	if _debug:
		if _lastUpdateTime == -1:
			_lastUpdateTime = time.time()

		while True:
			if _lastUpdateTime + _debugDelayPerFrame <= time.time():
				_lastUpdateTime += _debugDelayPerFrame
				_visualizer.update_geometry(_heMesh)
				break

			_visualizer.poll_events()
			_visualizer.update_renderer()


def debugChangeTriangleColor(halfEdgeId):
	global _debugTriangleColor, _debugRGBIndex, _debugRGBIncrease, _heMesh

	if _debug:
		if _debugRGBIncrease and _debugTriangleColor[_debugRGBIndex] >= 240:
			_debugTriangleColor[_debugRGBIndex] = 240
			_debugRGBIncrease = False
			_debugRGBIndex = (_debugRGBIndex - 1) % 3
		elif not _debugRGBIncrease and _debugTriangleColor[_debugRGBIndex] == 0:
			_debugTriangleColor[_debugRGBIndex] = 0
			_debugRGBIncrease = True
			_debugRGBIndex = (_debugRGBIndex + 2) % 3
		
		if _debugRGBIncrease:
			_debugTriangleColor[_debugRGBIndex] += _debugColorOffset
		else:
			_debugTriangleColor[_debugRGBIndex] -= _debugColorOffset

		triangleColor = [x / 255 for x in _debugTriangleColor]
		# triangleColor = [random.randint(0, 255) / 255, random.randint(0, 255) / 255, random.randint(0, 255) / 255]

		_heMesh.vertex_colors[getVertexId(halfEdgeId)] = triangleColor
		# _heMesh.vertex_colors[getNextVertexId(halfEdgeId)] = triangleColor
		# _heMesh.vertex_colors[getPreviousVertexId(halfEdgeId)] = triangleColor


def debugPrintInfos():
	print(f'\n##########   DEBUG   ##########')
	nbChar = len(_clers)

	print(f'nbTriangles = {len(_heMesh.triangles)}')
	print(f'nbChar = {nbChar}')

	print(f'nbHalfEdges = {_halfEdges.size}')
	
	C = _clers.count("C")
	L = _clers.count("L")
	E = _clers.count("E")
	R = _clers.count("R")
	S = _clers.count("S")

	print(f'nbVertices = {len(_heMesh.vertices)}')
	print(f'Identified as new during compression: {2 + C + L + E + R + S}/{nbChar}')

	print(f'C = {C}')
	print(f'c = {_clers.count("c")}')
	print(f'L = {L}')
	print(f'l = {_clers.count("l")}')
	print(f'E = {E}')
	print(f'e = {_clers.count("e")}')
	print(f'R = {R}')
	print(f'r = {_clers.count("r")}')
	print(f'S = {S}')
	print(f's = {_clers.count("s")}')

	borderVertexCounter = 0
	for he in _heMesh.half_edges:
		if he.is_boundary():
			borderVertexCounter += 1
	print(f'Border vertices: {borderVertexCounter}')
	print(f'#######  END OF DEBUG   #######\n')


# ------------------------------------------------------------
# Edgebreaker compression part
# ------------------------------------------------------------

# Initialize the data used by EdgeBreaker
def initData():
	global _marked, _inDeltas, _flagged, _halfEdges, _vertices, _triangles

	# Marks and flags
	_marked = [False] * len(_heMesh.vertices) 		# Marks: if a vertex has been visited or not
	_inDeltas = [False] * len(_heMesh.vertices) 	# Marks: if a vertex position has been saved in _deltas or not
	_flagged = [False] * len(_heMesh.triangles)		# Flags: if a triangle has been visited or not

	# Numpy arrays for acceleration
	_halfEdges = numpy.array(_heMesh.half_edges)
	_vertices = numpy.array(_heMesh.vertices)
	_triangles = numpy.array(_heMesh.triangles)

	# Mark boundary vertices as "seen"
	for halfEdge in _halfEdges:
		if halfEdge.twin == -1:
			_marked[halfEdge.vertex_indices[0]] = True
			_marked[halfEdge.vertex_indices[1]] = True


# Initialize the EdgeBreaker algorithm by choosing the best fitting starting vertex in the first mesh's triangle
def initCompression():
	global _startingHalfEdge, _previousHeId, _clers, _deltas

	initData()
	debugDrawAndWait()

	# Try to find a 'C' or an 'S' configuration by rotating the first vertex in the current triangle
	# If not possible, try to find an 'R' or an 'L' configuration with the same method
	for i in range(6):
		if i < 3:	# Search fo 'C' or 'S' configurations
			if getLeftCornerHeId(_startingHalfEdge) != -1 and getRightCornerHeId(_startingHalfEdge) != -1:
				if isMarked(_startingHalfEdge):
					break	# Found an 'S' configuration
				else:
					break	# Found a 'C' configuration
		else:		# Search fo 'R' or 'L' configurations
			if getLeftCornerHeId(_startingHalfEdge) != -1:
				break		# Found an 'R' configuration
			if getRightCornerHeId(_startingHalfEdge) != -1:
				break		# Found an 'L' configuration
		_startingHalfEdge = getNextHeId(_startingHalfEdge)

	first = getNextHeId(_startingHalfEdge)
	second = getPreviousHeId(_startingHalfEdge)
	third = _startingHalfEdge

	print("\n\n\nHalf edges:")
	print(f'first = {first}, HE = {_halfEdges[first]}')
	print(f'second = {second}, HE = {_halfEdges[second]}')
	print(f'third = {third}, HE = {_halfEdges[third]}')
	print("\n\n\n")

	# First vertex position
	addPosToDeltas(first)
	# Vector from first to second vertex
	addVectorToDeltas(first, second)

	# Mark these vertices as "seen"
	mark(first)
	mark(second)

	# Set previously visited half-edge
	_previousHeId = second

	# Draw if debug mode is actived
	debugChangeTriangleColor(first)
	debugDrawAndWait()
	debugChangeTriangleColor(second)
	debugDrawAndWait()


def compress(halfEdgeId):
	global _clers, _previousHeId

	while True:
		if halfEdgeId == -1:
			return

		if isFlagged(halfEdgeId):
			return

		flag(halfEdgeId)

		debugChangeTriangleColor(halfEdgeId)
		debugDrawAndWait()

		vertexId = getVertexId(halfEdgeId)
		fromTo = f'{getVertexId(_previousHeId)} → {vertexId}'

		if not isMarked(halfEdgeId):							# 'C' configuration
			if _inDeltas[vertexId]:
				debugPrint(f'{fromTo} Found c configuration')
				_clers += 'c'
			else:
				debugPrint(f'{fromTo} Found C configuration')
				_clers += 'C'
				addVectorToDeltas(_previousHeId, halfEdgeId)

			mark(halfEdgeId)
			_previousHeId = halfEdgeId
			halfEdgeId = getRightCornerHeId(halfEdgeId)

		else:
			if isFlagged(getRightCornerHeId(halfEdgeId)):	# isFlagged(i) == i triangle already seen
				if isFlagged(getLeftCornerHeId(halfEdgeId)):	# 'E' configuration
					if _inDeltas[vertexId]:
						debugPrint(f'{fromTo} Found e configuration')
						_clers += 'e'
					else:
						debugPrint(f'{fromTo} Found E configuration')
						_clers += 'E'
						addVectorToDeltas(_previousHeId, halfEdgeId)
					return

				else:											# 'R' configuration
					if _inDeltas[vertexId]:
						debugPrint(f'{fromTo} Found r configuration')
						_clers += 'r'
					else:
						debugPrint(f'{fromTo} Found R configuration')
						_clers += 'R'
						addVectorToDeltas(_previousHeId, halfEdgeId)
					_previousHeId = halfEdgeId
					halfEdgeId = getLeftCornerHeId(halfEdgeId)

			else:
				if isFlagged(getLeftCornerHeId(halfEdgeId)):	# 'L' configuration
					if _inDeltas[vertexId]:
						debugPrint(f'{fromTo} Found l configuration')
						_clers += 'l'
					else:
						debugPrint(f'{fromTo} Found L configuration')
						_clers += 'L'
						addVectorToDeltas(_previousHeId, halfEdgeId)
					_previousHeId = halfEdgeId
					halfEdgeId = getRightCornerHeId(halfEdgeId)

				else:											# 'S' configuration
					if _inDeltas[vertexId]:
						debugPrint(f'{fromTo} Found s configuration')
						_clers += 's'
					else:
						debugPrint(f'{fromTo} Found S configuration')
						_clers += 'S'
						addVectorToDeltas(_previousHeId, halfEdgeId)
					_previousHeId = halfEdgeId
					compress(getRightCornerHeId(halfEdgeId))	# Create a branch for the right triangles
					_previousHeId = halfEdgeId
					halfEdgeId = getLeftCornerHeId(halfEdgeId)	# When the right triangles are done, continue with the left triangles


# ------------------------------------------------------------
# Edgebreaker decompression part
# ------------------------------------------------------------

_V = []
_O = []
_G = []

_T = 0
_N = 2

_deltasIndex = 0
_clersIndex = 0


def readDeltas():
	global _deltasIndex

	delta = _deltas[_deltasIndex]
	_deltasIndex += 1
	return delta


def readClers():
	global _clersIndex

	letter = _clers[_clersIndex]
	_clersIndex += 1
	return letter


def next(c):
	if c % 3 == 2:
		return c - 2
	else:
		return c + 1

	
def previous(c):
	if c % 3 == 0:
		return c + 2
	else:
		return c - 1


def triangle(c):
	return int(c / 3)


def addVectors3D(v1, v2):
	return [v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]]


def decompressConnectivity(c):
	global _V, _O, _T, _N

	while True:
		_T += 1
		_O[c], _O[3 * _T] = 3 * _T, c
		_V[3 * _T + 1], _V[3 * _T + 2] = _V[previous(c)], _V[next(c)]
		c = next(_O[c])

		letter = readClers()
		cn = next(c)

		if letter == 'C' or letter == 'c':
			_O[cn] = -1
			_N += 1
			_V[3 * _T] = _N

		elif letter == 'L' or letter == 'l':
			_O[cn] = -2
			zip(cn)
			
		elif letter == 'R' or letter == 'r':
			_O[c] = -2
			c = cn

		elif letter == 'S' or letter == 's':
			decompressConnectivity(c)
			c = cn

		elif letter == 'E' or letter == 'e':
			_O[c] = -2
			_O[cn] = -2
			zip(cn)
			return


def zip(c):
	global _V, _O

	b = next(c)

	while _O[b] >= 0:
		b = next(_O[b])
	
	if _O[b] != -1:
		return
	_O[c], _O[b] = b, c

	a = previous(c)
	_V[previous(a)] = _V[previous(b)]

	while _O[a] >= 0 and b != a:
		a = previous(_O[a])
		_V[previous(a)] = _V[previous(b)]
	
	c = previous(c)

	while _O[c] >= 0 and c != b:
		c = previous(_O[c])
	
	if _O[c] == -2:
		zip(c)


def decompressVertices(c):
	global _G, _N

	while True:
		flag(triangle(c))
		if isMarked(_V[c]):
			_N += 1
			_G[_N] = _G # TODO: continue


def initDecompression():
	global _V, _O, _G, _T, _N, _marked, _flagged

	# Initialize arrays
	verticesCount = 2 + _clers.count('C') + _clers.count('L') + _clers.count('E') + _clers.count('R') + _clers.count('S')
	trianglesCount = len(_clers)
	halfEdgesCount = 3 * trianglesCount

	print(f'_vertices: {verticesCount}')
	print(f'_triangles: {trianglesCount}')
	print(f'_halfEdges: {halfEdgesCount}')

	_V = [0] * halfEdgesCount
	_V[1], _V[2] = 1, 2

	_O = [-3] * halfEdgesCount
	_O[0], _O[2] = -1, -1

	_T = 0
	_N = 2

	decompressConnectivity(1)

	_marked = [False] * verticesCount
	_flagged = [False] * trianglesCount

	_G = [[0, 0, 0]] * verticesCount
	_G[0] = readDeltas()
	_G[1] = addVectors3D(_G[0], readDeltas())
	_G[2] = addVectors3D(_G[1], readDeltas())
	_N = 2

	mark(0)
	mark(1)
	mark(2)
	flag(0)

	decompressVertices(_O[1])


# ------------------------------------------------------------
# Edgebreaker decompression test 2
# ------------------------------------------------------------

_verticesId = []			# V
_verticesLocation = []		# G
_oppositeHalfEdgeId = []	# O

_lastTriangle = 0			# T
_lastVertex = 2				# N
_lastDelta = 0

_markedVertices = []		# M
_markedTriangles = []		# U


def calculatePosFromDeltas(fromVertexPos):
	global _lastDelta

	_lastDelta += 1
	vector = _deltas[_lastDelta]

	return [fromVertexPos[0] + vector[0], fromVertexPos[1] + vector[1], fromVertexPos[2] + vector[2]]


def decompressConnectivity(halfEdgeId):
	...


def initDecompression():
	global _verticesId, _verticesLocation, _oppositeHalfEdgeId, _lastTriangle, _lastVertex, _markedVertices, _markedTriangles

	# Initialize arrays
	verticesCount = 2 + _clers.count('C') + _clers.count('L') + _clers.count('E') + _clers.count('R') + _clers.count('S')
	trianglesCount = len(_clers)
	halfEdgesCount = 3 * trianglesCount

	print(f'_vertices: {verticesCount}')
	print(f'_triangles: {trianglesCount}')
	print(f'_halfEdges: {halfEdgesCount}')

	_verticesId = [0] * verticesCount
	_verticesId[1], _verticesId[2] = 1, 2

	_oppositeHalfEdgeId = [-3] * halfEdgesCount
	_oppositeHalfEdgeId[0], _oppositeHalfEdgeId[2] = -1, -1

	_lastTriangle = 0
	_lastVertex = 2

	# Decompress connecivity
	# decompressConnectivity(1)

	# Initialize marks
	_markedVertices = [False] * verticesCount
	_markedTriangles = [False] * verticesCount

	# Read first triangle
	_verticesLocation[0] = _deltas[0]
	_verticesLocation[1] = readDeltas(_verticesLocation[0])
	_verticesLocation[2] = readDeltas(_verticesLocation[1])

	_lastVertex = 2

	_markedVertices[0], _markedVertices[1], _markedVertices[2] = True, True, True
	_markedTriangles[0] = True

	# Decompress vertices
	# decompressVertices(_oppositeHalfEdgeId[1])


# ------------------------------------------------------------
# Edgebreaker decompression test 1
# ------------------------------------------------------------

## TRIANGLE DATACLASS

@dataclass
class Triangle:
	id: int
	rightHe: open3d.geometry.HalfEdge
	leftHe: open3d.geometry.HalfEdge
	bottomHe: open3d.geometry.HalfEdge


## EDGEBREAKER DECOMPRESSION RELATED

_letterIndex = 0

_halfEdgeId = 0
_vertexId = 0
_triangleId = 0

_halfEdgeStack = []


## FUNCTIONS

def readDeltas(fromVertexId):
	global _vertexId

	_vertices[_vertexId] = _vertices[fromVertexId] + _deltas[_vertexId]
	_vertexId += 1


def createHalfEdge(fromId, toId, triangleId = -1, nextHeId = -1, twinHeId = -1):
	global _halfEdges, _halfEdgeId

	halfEdge = open3d.geometry.HalfEdge()
	halfEdge.vertex_indices = [fromId, toId]
	if triangleId != -1:
		halfEdge.triangle_index = triangleId
	if nextHeId != -1:
		halfEdge.next = nextHeId
	if twinHeId != -1:
		halfEdge.twin = twinHeId

	_halfEdges[_halfEdgeId] = halfEdge
	_halfEdgeId += 1
	
	return halfEdge


def createTriangle(halfEdgeId, direction):
	global _halfEdges, _vertices, _triangles, _halfEdgeStack, _letterIndex, _vertexId, _triangleId

	halfEdge = getHalfEdge(halfEdgeId)

	# Create half-edges
	bottomHe = createHalfEdge(halfEdge.vertex_indices[1], halfEdge.vertex_indices[0], _triangleId, _halfEdgeId + 1)
	rightHe = createHalfEdge(halfEdge.vertex_indices[0], _vertexId, _triangleId, _halfEdgeId + 1)
	leftHe = createHalfEdge(_vertexId, halfEdge.vertex_indices[1], _triangleId, _halfEdgeId - 2)

	# Link bottom half-edge to previous triangle
	bottomHe.twin = halfEdgeId
	halfEdge.twin = _halfEdgeId - 3

	# Get position of top vertex
	readDeltas(halfEdge.vertex_indices[1])

	# Create the triangle
	_triangles[_triangleId] = [0, 1, 2]


# TODO: generalize
# Currently only working for 'C' initial triangles
def initDecompression():
	global _halfEdges, _vertices, _triangles, _halfEdgeStack, _letterIndex, _vertexId, _triangleId

	verticesCount = 2 + _clers.count('C') + _clers.count('L') + _clers.count('E') + _clers.count('R') + _clers.count('S')
	_vertices = numpy.full((verticesCount, 3), [0, 0, 0])

	triangleCount = len(_clers)
	_triangles = numpy.full((triangleCount, 3), [-1, -1, -1])
	_halfEdges = numpy.full(triangleCount * 3, None, dtype=open3d.geometry.HalfEdge)

	print(f'_vertices size: {int(_vertices.size / 3)}')
	print(f'_triangles size: {int(_triangles.size / 3)}')
	print(f'_halfEdges size: {_halfEdges.size}')

	# Get vertices positions
	_vertices[0] = _deltas[0]
	_vertices[1] = _vertices[0] + _deltas[1]
	_vertices[2] = _vertices[1] + _deltas[2]

	# Create the triangle
	_triangles[0] = [0, 1, 2]

	# Create the according half-edges
	createHalfEdge(0, 1, 0, 1)
	createHalfEdge(1, 2, 0, 2)
	createHalfEdge(2, 0, 0, 0)

	# Add edgebreaker working half-edge to the stack
	_halfEdgeStack.append(_halfEdges[1])

	# Update the edgebreaker decompression indexes
	_letterIndex = 1	# Read 1 letter
	_vertexId = 3		# Created 3 vertices
	_triangleId = 1		# Created 1 triangle
	# _halfEdgeId automatically updates in createHalfEdge() function


def saveTriangle():
	# print(f'f {_stack[2]} {_stack[1]} {_stack[0]}')	# Vertices: right -> top -> left
	...


def decompress():
	global _halfEdges, _vertices, _triangles, _halfEdgeStack, _letterIndex, _vertexId, _triangleId

	while True:
		if _letterIndex == len(_clers):
			break

		letter = _clers[_letterIndex]
		_letterIndex += 1

		print()
		print(f'Letter: {letter}')

		if letter == 'C':
			...
		elif letter == 'c':
			print(f'ERROR: should not find \'c\' chars in CLERS string')
		elif letter == 'L':
			...
		elif letter == 'l':
			...
		elif letter == 'E':
			...
		elif letter == 'e':
			...
		elif letter == 'R':
			...
		elif letter == 'r':
			...
		elif letter == 'S':
			isInRightBranch = True
			...
		elif letter == 's':
			...


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def doProfiling():
	if _doProfiling:
		with cProfile.Profile() as pr:
			main()
		
		with open('profiling_stats.txt', 'w') as stream:
			stats = Stats(pr, stream=stream)
			stats.strip_dirs()
			stats.sort_stats('time')
			stats.dump_stats('.prof_stats')
			stats.print_stats()

	return 0


def main():
	global _heMesh

	print(f'\n\n\n\n\nRunning MAIN from EdgeBreaker.py')
	print(f'Starting at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')
	mesh = open3d.io.read_triangle_mesh("Models/complex_shape_2.obj")
	# Quantization.quantizeVertices(mesh, 4)
	_heMesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)
	
	debugInit()
	initCompression()
	compress(_startingHalfEdge)

	# print(f'CLERS = {_clers}')
	# print(f'Deltas: {len(_deltas)}')
	# for v in _deltas:
	# 	print(v)

	debugPrintInfos()
	
	initDecompression()
	# decompress()

	debugEnd()

	return 0


if __name__ == '__main__':
	if _doProfiling:
		sys.exit(doProfiling())
	else:
		sys.exit(main())
