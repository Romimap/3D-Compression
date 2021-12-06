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

_debug = True						# True if you want to enable color changes and delay between draw calls
_debugPrint = False					# True if you want to enable debug prints

_debugDelayPerFrame = 0.01			# Delay between draw calls

_debugTriangleColor = [240, 0, 0]	# The current triangle color for debug-drawing triangles (to use, divide by 255)
_debugColorOffset = 24				# For each triangle, 24 will be added or removed from one of the RGB component
_debugRGBIndex = 1					# RGB index, 0 = R, 1 = G, 2 = B
_debugRGBIncrease = True			# When true, we add _debugColorOffset for each triangle, else we subtract _debugColorOffset

_visualizer = None					# The visualizer (the window)
_lastUpdateTime = -1				# Last visual update time in seconds


## DATA STORAGE

# Store frequently accessed data in numpy arrays to accelerate access time
_halfEdges = None
_vertices = None
_triangles = None


## EDGEBREAKER RELATED

_heMesh = None 				# The mesh containing half-edges data

_clers = ""					# String storing the CLERS steps of the EdgeBreaker algorithm's path
_deltas = []				# List of 3D points/vectors storing the first points and the correction vectors
_normals = []				# TODO

_marked = []				# List of bool indicating whether a vertex has already been visited: M in the paper
_flagged = []				# List of bool indicating whether a triangle has already been visited: U in the paper


## EDGEBREAKER COMPRESSION SPECIFIC

_startingHalfEdge = 0		# Select first half-edge to begin the EdgeBreaker algorithm
_previousHeId = -1			# Id of the previously visited half-edge, used to calculate delta vector(_previousHeId → halfEdgeId)


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


# ------------------------------------------------------------
# DELTAS
# ------------------------------------------------------------

def addPosToDeltas(halfEdgeId):
	global _deltas

	pos = getVertexPosFromHeId(halfEdgeId)
	_deltas.append(pos)
	# debugPrint(f'# Add pos {pos}')


def addVectorToDeltas(previousHalfEdgeId, halfEdgeId):
	global _deltas

	vector = getDistanceVectorFromHeId(previousHalfEdgeId, halfEdgeId)
	_deltas.append(vector)
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
	print(f'L = {L}')
	print(f'E = {E}')
	print(f'R = {R}')
	print(f'S = {S}')

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
	global _marked, _flagged, _halfEdges, _vertices, _triangles

	# Marks and flags
	_marked = [False] * len(_heMesh.vertices) 		# Marks: if a vertex has been visited or not
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

	first = 0
	second = 1
	third = 2

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


def compressRecursive(halfEdgeId = 2):
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
			debugPrint(f'{fromTo} Found C configuration')
			_clers += 'C'
			addVectorToDeltas(_previousHeId, halfEdgeId)

			mark(halfEdgeId)
			_previousHeId = halfEdgeId
			halfEdgeId = getRightCornerHeId(halfEdgeId)

		else:
			if isFlagged(getRightCornerHeId(halfEdgeId)):	# isFlagged(i) == i triangle already seen
				if isFlagged(getLeftCornerHeId(halfEdgeId)):	# 'E' configuration
					debugPrint(f'{fromTo} Found E configuration')
					_clers += 'E'
					addVectorToDeltas(_previousHeId, halfEdgeId)
					return

				else:											# 'R' configuration
					debugPrint(f'{fromTo} Found R configuration')
					_clers += 'R'
					addVectorToDeltas(_previousHeId, halfEdgeId)
					_previousHeId = halfEdgeId
					halfEdgeId = getLeftCornerHeId(halfEdgeId)

			else:
				if isFlagged(getLeftCornerHeId(halfEdgeId)):	# 'L' configuration
					debugPrint(f'{fromTo} Found L configuration')
					_clers += 'L'
					addVectorToDeltas(_previousHeId, halfEdgeId)
					_previousHeId = halfEdgeId
					halfEdgeId = getRightCornerHeId(halfEdgeId)

				else:											# 'S' configuration
					debugPrint(f'{fromTo} Found S configuration')
					_clers += 'S'
					addVectorToDeltas(_previousHeId, halfEdgeId)
					_previousHeId = halfEdgeId
					compressRecursive(getRightCornerHeId(halfEdgeId))	# Create a branch for the right triangles
					_previousHeId = halfEdgeId
					halfEdgeId = getLeftCornerHeId(halfEdgeId)	# When the right triangles are done, continue with the left triangles


def compress():
	print(f'Edgebreaker compression starting at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')

	initCompression()
	compressRecursive()

	print(f'Edgebreaker compression ending at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')

	return _clers, _deltas, _normals


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
	mesh = open3d.io.read_triangle_mesh("Models/Sphere.obj")
	# Quantization.quantizeVertices(mesh, 4)
	_heMesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)
	
	debugInit()
	
	clers, deltas, normals = compress()

	# print(f'CLERS = {clers}')
	# print(f'Deltas: {len(deltas)}')
	# for v in _deltas:
	# 	print(v)

	debugPrintInfos()

	debugEnd()

	return 0


if __name__ == '__main__':
	if _doProfiling:
		sys.exit(doProfiling())
	else:
		sys.exit(main())
