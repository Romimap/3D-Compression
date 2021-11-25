'''
Python EdgeBreaker algorithm working with the Open3D library.

Implemented and improved from:
https://www.cs.cmu.edu/~alla/edgebreaker_simple.pdf
'''

import numpy
import open3d
import sys
import time


# ------------------------------------------------------------
# Global variables
# ------------------------------------------------------------

## DEBUG AND VISUALIZATION

_debug = False						# True if you want to enable color changes and delay between draw calls
_debugPrint = False					# True if you want to enable debug prints

_debugDelayPerFrame = 0.1			# Delay between draw calls

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
_flagged = []				# List of bool indicating whether a triangle has already been visited: U in the paper

_missingTrianglesCount = 0	# The number of triangles not already seen by EdgeBreaker ## UNUSED FOR NOW ##

_startingHalfEdge = 0		# Select first half-edge to begin the EdgeBreaker algorithm


# ------------------------------------------------------------
# Data access functions
# ------------------------------------------------------------

## VERTICES

def getVertexId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return _heMesh.half_edges[halfEdgeId].vertex_indices[0]


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
	return _heMesh.half_edges[halfEdgeId].next


def getPreviousHeId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return getNextHeId(getNextHeId(halfEdgeId))


def getTwinHeId(halfEdgeId):
	if halfEdgeId == -1:
		return -1
	return _heMesh.half_edges[halfEdgeId].twin


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
	return _heMesh.half_edges[halfEdgeId].triangle_index


## DISTANCE VECTORS

def getDistanceVectorFromVerticesId(fromVertexId, toVertexId):
	return _heMesh.vertices[toVertexId] - _heMesh.vertices[fromVertexId]


def getDistanceVectorFromHeId(fromHeId, toHeId):
	fromVertexId = getVertexId(fromHeId)
	toVertexId = getVertexId(toHeId)
	return getDistanceVectorFromVerticesId(fromVertexId, toVertexId)


## VERTEX POSITION

def getVertexPosFromVertexId(vertexId):
	return _heMesh.vertices[vertexId]


def getVertexPosFromHeId(halfEdgeId):
	return _heMesh.vertices[getVertexId(halfEdgeId)]


## MARKS/FLAGS

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


# ------------------------------------------------------------
# Debug functions
# ------------------------------------------------------------

def printMeshInfo():
	mesh = open3d.io.read_triangle_mesh("Models/cube.obj")
	he_mesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)
	print("Half edges:")
	print(he_mesh.half_edges)


def debugInit():
	global _debug, _visualizer

	_debug = True

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
# Other functions
# ------------------------------------------------------------

# Initialize the data used by EdgeBreaker
def initData():
	global _marked, _flagged, _missingTrianglesCount

	# Marks and flags
	_marked = [False] * len(_heMesh.vertices) 		# Marks: if a vertex has been visited or not
	_flagged = [False] * len(_heMesh.triangles)		# Flags: if a triangle has been visited or not
	_missingTrianglesCount = len(_heMesh.triangles)	# No triangle has been seen yet

	# Mark boundary vertices as "seen"
	for halfEdge in _heMesh.half_edges:
		if halfEdge.twin == -1:
			_marked[halfEdge.vertex_indices[0]] = True
			_marked[halfEdge.vertex_indices[1]] = True


# Initialize the EdgeBreaker algorithm by choosing the best fitting starting vertex in the first mesh's triangle
def initCompression():
	global _startingHalfEdge, _clers, _deltas

	initData()

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

	# First vertex position
	_deltas.append(getVertexPosFromHeId(getPreviousHeId(_startingHalfEdge)))
	# Vector from first to second vertex
	_deltas.append(getDistanceVectorFromHeId(getPreviousHeId(_startingHalfEdge), _startingHalfEdge))
	# Vector from second to third vertex
	_deltas.append(getDistanceVectorFromHeId(_startingHalfEdge, getNextHeId(_startingHalfEdge)))

	# Mark these vertices as "seen"
	mark(getNextHeId(_startingHalfEdge))
	mark(getPreviousHeId(_startingHalfEdge))


def compress(halfEdgeId):
	global _deltas, _clers

	debugDrawAndWait()

	while True:
		if halfEdgeId == -1:
			return

		debugPrint(f'\nCurrent HE id: {halfEdgeId}')
		debugPrint(f'Current vertex id: {getVertexId(halfEdgeId)}')
		debugPrint(f'Marks: {_marked}')
		debugPrint(f'Flags: {_flagged}')

		flag(halfEdgeId)

		debugChangeTriangleColor(halfEdgeId)
		debugDrawAndWait()

		if not isMarked(halfEdgeId):							# 'C' configuration
			print("Found C configuration")
			# Append correction vector
			_deltas.append(getVertexPosFromHeId(halfEdgeId)								# Current
							- getVertexPosFromHeId(getPreviousHeId(halfEdgeId))			# Previous
							- getVertexPosFromHeId(getNextHeId(halfEdgeId))				# Next
							+ getVertexPosFromHeId(getOppositeCornerHeId(halfEdgeId)))	# Opposite
			_clers += 'C'
			mark(halfEdgeId)
			halfEdgeId = getRightCornerHeId(halfEdgeId)
		else:

			if isFlagged(getRightCornerHeId(halfEdgeId)):	# isFlagged(i) == i triangle already seen
				if isFlagged(getLeftCornerHeId(halfEdgeId)):	# 'E' configuration
					print("Found E configuration")
					_clers += 'E'
					return
				else:											# 'R' configuration
					print("Found R configuration")
					_clers += 'R'
					halfEdgeId = getLeftCornerHeId(halfEdgeId)
			else:
				if isFlagged(getLeftCornerHeId(halfEdgeId)):	# 'L' configuration
					print("Found L configuration")
					_clers += 'L'
					halfEdgeId = getRightCornerHeId(halfEdgeId)
				else:											# 'S' configuration
					print("Found S configuration")
					_clers += 'S'
					compress(getRightCornerHeId(halfEdgeId))	# Create a branch for the right triangles
					halfEdgeId = getLeftCornerHeId(halfEdgeId)	# When the right triangles are done, continue with the left triangles


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():
	global _heMesh

	print("\n\n\n\n\nRunning MAIN from EdgeBreaker.py")
	mesh = open3d.io.read_triangle_mesh("Models/bunny.obj")
	_heMesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)
	
	debugInit()
	initCompression()
	compress(_startingHalfEdge)

	print(f'CLERS = {_clers}')
	debugPrint("Deltas:")
	for v in _deltas:
		debugPrint(v)

	# print(numpy.asarray(_heMesh.triangles))
	# print(numpy.asarray(_heMesh.half_edges))
	debugEnd()

	return 0


if __name__ == '__main__':
	sys.exit(main())
