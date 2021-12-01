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

_debugDelayPerFrame = 1			# Delay between draw calls

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

_missingTrianglesCount = 0	# The number of triangles not already seen by EdgeBreaker ## UNUSED FOR NOW ##

_startingHalfEdge = 71		# Select first half-edge to begin the EdgeBreaker algorithm
_previousHeId = -1			# Id of the previously visited half-edge, used to calculate delta vector(_previousHeId → halfEdgeId)


## EDGEBREAKER DECOMPRESSION RELATED

_stack = []

_vertexIndex = 3
_letterIndex = 0

_addedInC = []
_keepForSLeftBranch = []


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

def addPosToDeltas(halfEdgeId):
	global _deltas, _inDeltas

	vertexId = getVertexId(halfEdgeId)
	pos = getVertexPosFromHeId(halfEdgeId)
	_deltas.append(pos)
	_inDeltas[vertexId] = True
	debugPrint(f'# Add pos {pos}')


def addVectorToDeltas(previousHalfEdgeId, halfEdgeId):
	global _deltas, _inDeltas

	vertexId = getVertexId(halfEdgeId)
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


def debugPrintInfos():
	print(f'\n##########   DEBUG   ##########')
	nbChar = len(_clers)

	print(f'nbTriangles = {len(_heMesh.triangles)}')
	print(f'nbChar = {nbChar}')
	
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
	print(f'#######  END OF DEBUG   #######\n')


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
	global _deltas, _clers, _previousHeId, clersOcc

	while True:
		if halfEdgeId == -1:
			return

		if isFlagged(halfEdgeId):
			return

		flag(halfEdgeId)

		debugChangeTriangleColor(halfEdgeId)
		debugDrawAndWait()

		fromTo = f'{getVertexId(_previousHeId)} → {getVertexId(halfEdgeId)}'
		vertexId = getVertexId(halfEdgeId)

		if not isMarked(halfEdgeId):							# 'C' configuration
			if _inDeltas[vertexId]:
				print(f'{fromTo} Found c configuration')
				_clers += 'c'
			else:
				print(f'{fromTo} Found C configuration')
				_clers += 'C'
				addVectorToDeltas(_previousHeId, halfEdgeId)

			mark(halfEdgeId)
			_previousHeId = halfEdgeId
			halfEdgeId = getRightCornerHeId(halfEdgeId)

		else:
			if isFlagged(getRightCornerHeId(halfEdgeId)):	# isFlagged(i) == i triangle already seen
				if isFlagged(getLeftCornerHeId(halfEdgeId)):	# 'E' configuration
					if _inDeltas[vertexId]:
						print(f'{fromTo} Found e configuration')
						_clers += 'e'
					else:
						print(f'{fromTo} Found E configuration')
						_clers += 'E'
						addVectorToDeltas(_previousHeId, halfEdgeId)
					return

				else:											# 'R' configuration
					if _inDeltas[vertexId]:
						print(f'{fromTo} Found r configuration')
						_clers += 'r'
					else:
						print(f'{fromTo} Found R configuration')
						_clers += 'R'
						addVectorToDeltas(_previousHeId, halfEdgeId)
					_previousHeId = halfEdgeId
					halfEdgeId = getLeftCornerHeId(halfEdgeId)

			else:
				if isFlagged(getLeftCornerHeId(halfEdgeId)):	# 'L' configuration
					if _inDeltas[vertexId]:
						print(f'{fromTo} Found l configuration')
						_clers += 'l'
					else:
						print(f'{fromTo} Found L configuration')
						_clers += 'L'
						addVectorToDeltas(_previousHeId, halfEdgeId)
					_previousHeId = halfEdgeId
					halfEdgeId = getRightCornerHeId(halfEdgeId)

				else:											# 'S' configuration
					if _inDeltas[vertexId]:
						print(f'{fromTo} Found s configuration')
						_clers += 's'
					else:
						print(f'{fromTo} Found S configuration')
						_clers += 'S'
						addVectorToDeltas(_previousHeId, halfEdgeId)
					_previousHeId = halfEdgeId
					compress(getRightCornerHeId(halfEdgeId))	# Create a branch for the right triangles
					_previousHeId = halfEdgeId
					halfEdgeId = getLeftCornerHeId(halfEdgeId)	# When the right triangles are done, continue with the left triangles


# ------------------------------------------------------------
# Edgebreaker decompression part
# ------------------------------------------------------------

def createNewVertex(isFromCCase = False):
	global _stack, _vertexIndex

	_stack.insert(1, _vertexIndex)
	print(f'v {_vertexIndex}')
	if isFromCCase:
		_addedInC.append(_vertexIndex)
	_vertexIndex += 1


def getExistingVertexFromEnd():
	global _stack

	_stack.insert(1, _stack.pop(-1))


def getExistingVertexFromBegining():
	global _stack

	_stack[1], _stack[2] = _stack[2], _stack[1]


def removeVertexIfNotSaved(index):
	global _stack, _keepForSLeftBranch

	id = _keepForSLeftBranch.index(_stack[index])
	if id == ValueError:	# Not found in _keepForSLeftBranch, can remove it from _stack
		del(_stack[index])
	else:					# Found in _keepForSLeftBranch, remove from _keepForSLeftBranch but has to remain in _stack for later use
		del(_keepForSLeftBranch[id])


def removeVertexInRCase():
	removeVertexIfNotSaved(0)	# Right vertex


def removeVertexInLCase():
	removeVertexIfNotSaved(2)	# Left vertex


def removeVerticesInECase():
	removeVertexIfNotSaved(2)	# Left vertex
	removeVertexIfNotSaved(1)	# Top vertex
	removeVertexIfNotSaved(0)	# Right vertex


def saveVerticesInSCase():
	_keepForSLeftBranch

	_keepForSLeftBranch.append(_stack[1])	# Top vertex
	_keepForSLeftBranch.append(_stack[2])	# Left vertex


def saveTriangle():
	print(f'f {_stack[2]} {_stack[1]} {_stack[0]}')	# Vertices: right -> top -> left


def initDecompression():
	global _stack

	_stack = [2, 1]
	print(f'v 1')
	print(f'v 2')


def decompress():
	global _stack, _vertexIndex, _letterIndex

	isInRightBranch = True

	while True:
		if _letterIndex == len(_clers):
			break

		letter = _clers[_letterIndex]
		_letterIndex += 1

		print()
		print(f'Letter: {letter}')
		print(f'Stack before: {_stack}')
		print(f'Saved before: {_keepForSLeftBranch}')
		print(f'C indexes before: {_addedInC}')

		if letter == 'C':
			createNewVertex(isFromCCase=True)
			saveTriangle()
		elif letter == 'L':
			...
		elif letter == 'E':
			isInRightBranch = False
			...
		elif letter == 'R':
			if isInRightBranch:
				getExistingVertexFromEnd()
				saveTriangle()
				removeVertexInRCase()
			else:
				createNewVertex()

			...
		elif letter == 'S':
			isInRightBranch = True
			...
		
		print(f'Stack after: {_stack}')


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
	mesh = open3d.io.read_triangle_mesh("Models/complex_shape_3.obj")
	_heMesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)
	
	debugInit()
	initCompression()
	compress(_startingHalfEdge)

	print(f'CLERS = {_clers}')
	# print(f'Deltas: {len(_deltas)}')
	# for v in _deltas:
	# 	print(v)

	debugPrintInfos()
	
	# initDecompression()
	# decompress()

	debugEnd()

	return 0


if __name__ == '__main__':
	if _doProfiling:
		sys.exit(doProfiling())
	else:
		sys.exit(main())
