'''
Python EdgeBreaker algorithm working with the Open3D library.

Implemented and improved from:
https://www.cs.cmu.edu/~alla/edgebreaker_simple.pdf
'''

import cProfile
import numpy
import open3d
import sys
import time

from pstats import Stats, SortKey


# ------------------------------------------------------------
# Global variables
# ------------------------------------------------------------

## DEBUG AND VISUALIZATION

_doProfiling = False				# Do some profiling

_debug = True						# True if you want to enable color changes and delay between draw calls
_debugPrint = False					# True if you want to enable debug prints

_debugDelayPerFrame = 0.01			# Delay between draw calls

_debugTriangleColor = [240, 0, 0]	# The current triangle color for debug-drawing triangles (to use, divide by 255)
_debugColorOffset = 24				# For each triangle, 24 will be added or removed from one of the RGB component
_debugRGBIndex = 1					# RGB index, 0 = R, 1 = G, 2 = B
_debugRGBIncrease = True			# When true, we add _debugColorOffset for each triangle, else we subtract _debugColorOffset

_visualizer = None					# The visualizer (the window)
_lastUpdateTime = -1				# Last visual update time in seconds


## EDGEBREAKER RELATED

_heMesh = None 				# The mesh containing half-edges data

_deltas = []				# List of 3D points/vectors storing the first points and the correction vectors
_clers = ""					# String storing the CLERS steps of the EdgeBreaker algorithm's path

_marked = []				# List of bool indicating whether a vertex has already been visited: M in the paper
_inDeltas = []				# List of bool indicating whether a vertex position has already been saved in _deltas array
_flagged = []				# List of bool indicating whether a triangle has already been visited: U in the paper

_missingTrianglesCount = 0	# The number of triangles not already seen by EdgeBreaker ## UNUSED FOR NOW ##

_startingHalfEdge = 0		# Select first half-edge to begin the EdgeBreaker algorithm
_previousHeId = -1			# Id of the previously visited half-edge, used to calculate delta vector(_previousHeId → halfEdgeId)


## EDGEBREAKER DECOMPRESSION RELATED

_stack = []

_vertexIndex = 3
_letterIndex = 0

_returnWhenL = True


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

def addPosToDeltasIfNecessary(halfEdgeId):
	global _deltas, _inDeltas

	vertexId = getVertexId(halfEdgeId)
	if not _inDeltas[vertexId]:
		pos = getVertexPosFromHeId(halfEdgeId)
		_deltas.append(pos)
		_inDeltas[vertexId] = True
		debugPrint(f'# Add pos {pos}')


def addVectorToDeltasIfNecessary(previousHalfEdgeId, halfEdgeId):
	global _deltas, _inDeltas

	vertexId = getVertexId(halfEdgeId)
	if not _inDeltas[vertexId]:
		vector = getDistanceVectorFromHeId(previousHalfEdgeId, halfEdgeId)
		_deltas.append(vector)
		_inDeltas[vertexId] = True
		debugPrint(f'# Add vector {vector}')


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

		_heMesh.vertex_colors[getVertexId(halfEdgeId)] = triangleColor
		# _heMesh.vertex_colors[getNextVertexId(halfEdgeId)] = triangleColor
		# _heMesh.vertex_colors[getPreviousVertexId(halfEdgeId)] = triangleColor


# ------------------------------------------------------------
# Edgebreaker compression part
# ------------------------------------------------------------

# Initialize the data used by EdgeBreaker
def initData():
	global _marked, _inDeltas, _flagged, _missingTrianglesCount, _halfEdges, _vertices, _triangles

	# Marks and flags
	_marked = [False] * len(_heMesh.vertices) 		# Marks: if a vertex has been visited or not
	_inDeltas = [False] * len(_heMesh.vertices) 	# Marks: if a vertex position has been saved in _deltas or not
	_flagged = [False] * len(_heMesh.triangles)		# Flags: if a triangle has been visited or not
	_missingTrianglesCount = len(_heMesh.triangles)	# No triangle has been seen yet

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
	global _startingHalfEdge, _previousHeId, _clers, _deltas, _inDeltas

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

	# First vertex position
	addPosToDeltasIfNecessary(first)
	# Vector from first to second vertex
	addVectorToDeltasIfNecessary(first, second)

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
	global _deltas, _clers, _previousHeId

	while True:
		if halfEdgeId == -1:
			return

		if isFlagged(halfEdgeId):
			return

		flag(halfEdgeId)

		debugChangeTriangleColor(halfEdgeId)
		debugDrawAndWait()

		fromTo = f'{getVertexId(_previousHeId)} → {getVertexId(halfEdgeId)}'

		if not isMarked(halfEdgeId):							# 'C' configuration
			debugPrint(f'{fromTo} Found C configuration')
			# Append correction vector
			addVectorToDeltasIfNecessary(_previousHeId, halfEdgeId)
			# _deltas.append(getVertexPosFromHeId(halfEdgeId)							# Current
			# 				- getVertexPosFromHeId(getPreviousHeId(halfEdgeId))			# Previous
			# 				- getVertexPosFromHeId(getNextHeId(halfEdgeId))				# Next
			# 				+ getVertexPosFromHeId(getOppositeCornerHeId(halfEdgeId)))	# Opposite
			_clers += 'C'
			mark(halfEdgeId)
			_previousHeId = halfEdgeId
			halfEdgeId = getRightCornerHeId(halfEdgeId)
		else:

			if isFlagged(getRightCornerHeId(halfEdgeId)):	# isFlagged(i) == i triangle already seen
				if isFlagged(getLeftCornerHeId(halfEdgeId)):	# 'E' configuration
					debugPrint(f'{fromTo} Found E configuration')
					addVectorToDeltasIfNecessary(_previousHeId, halfEdgeId)
					_clers += 'E'
					return
				else:											# 'R' configuration
					debugPrint(f'{fromTo} Found R configuration')
					addVectorToDeltasIfNecessary(_previousHeId, halfEdgeId)
					_clers += 'R'
					_previousHeId = halfEdgeId
					halfEdgeId = getLeftCornerHeId(halfEdgeId)
			else:
				if isFlagged(getLeftCornerHeId(halfEdgeId)):	# 'L' configuration
					debugPrint(f'{fromTo} Found L configuration')
					addVectorToDeltasIfNecessary(_previousHeId, halfEdgeId)
					_clers += 'L'
					_previousHeId = halfEdgeId
					halfEdgeId = getRightCornerHeId(halfEdgeId)
				else:											# 'S' configuration
					debugPrint(f'{fromTo} Found S configuration')
					addVectorToDeltasIfNecessary(_previousHeId, halfEdgeId)
					_clers += 'S'
					_previousHeId = halfEdgeId
					compress(getRightCornerHeId(halfEdgeId))	# Create a branch for the right triangles
					_previousHeId = halfEdgeId
					halfEdgeId = getLeftCornerHeId(halfEdgeId)	# When the right triangles are done, continue with the left triangles


# ------------------------------------------------------------
# Edgebreaker decompression part
# ------------------------------------------------------------

def initDecompression():
	global _stack

	_stack = [2, 1]
	print(f'v 1')
	print(f'v 2')


def readFromDeltas():
	global _stack, _vertexIndex

	print(f'v {_vertexIndex}')
	_vertexIndex += 1
	_stack = [_stack[0] + _vertexIndex + _stack[1]]


def decompress():
	global _stack, _vertexIndex, _letterIndex, _returnWhenL

	while True:
		if _letterIndex == len(_clers):
			break

		letter = _clers[_letterIndex]
		_letterIndex += 1

		print()
		print(f'Letter: {letter}')
		print(f'Stack: {_stack}')

		if letter == 'C':
			readFromDeltas()

			print(f'C f {_stack[0]} {_stack[1]} {_stack[2]}')

			leftOverVertexIndex = _stack[2]		# Left vertex
			_stack = [_stack[0], _stack[1]]		# Right edge

			previousReturnWhenL = _returnWhenL
			_returnWhenL = True
			decompress()
			_returnWhenL = previousReturnWhenL

			_stack = [_stack[0] + leftOverVertexIndex + _stack[1]]	# Right, top, left
		elif letter == 'L':
			if _returnWhenL:
				...
			else:
				readFromDeltas()

				print(f'L f {_stack[0]} {_stack[1]} {_stack[2]}')

				_stack = [_stack[0], _stack[1]]		# Right edge
		elif letter == 'E':
			readFromDeltas()
			
			print(f'E f {_stack[0]} {_stack[1]} {_stack[2]}')
			...
		elif letter == 'R':
			readFromDeltas()

			print(f'R f {_stack[0]} {_stack[1]} {_stack[2]}')
			
			_stack = [_stack[1], _stack[2]]		# Left edge
		elif letter == 'S':
			readFromDeltas()

			print(f'S f {_stack[0]} {_stack[1]} {_stack[2]}')

			leftEdge = [_stack[1], _stack[2]]	# Left vertex
			_stack = [_stack[0], _stack[1]]		# Right edge

			previousReturnWhenL = _returnWhenL
			_returnWhenL = False
			decompress()
			_returnWhenL = previousReturnWhenL

			_stack = leftEdge	# Left edge


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

	print("\n\n\n\n\nRunning MAIN from EdgeBreaker.py")
	mesh = open3d.io.read_triangle_mesh("Models/complex_shape_2.obj")
	_heMesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)
	
	debugInit()
	initCompression()
	compress(_startingHalfEdge)

	print(f'CLERS = {_clers}')
	print(f'Deltas: {len(_deltas)}')
	for v in _deltas:
		print(v)
	
	# initDecompression()
	# decompress()

	debugEnd()

	return 0


if __name__ == '__main__':
	if _doProfiling:
		sys.exit(doProfiling())
	else:
		sys.exit(main())
