'''
Python EdgeBreaker algorithm working with the Open3D library.

Implemented from:
https://www.cs.cmu.edu/~alla/edgebreaker_simple.pdf
'''

import sys
import open3d
import numpy


# ------------------------------------------------------------
# Global variables
# ------------------------------------------------------------

_heMesh = None 				# The mesh containing half-edges data

_deltas = []				# List of 3D points/vectors storing the first points and the correction vectors
_clers = ""					# String storing the CLERS steps of the EdgeBreaker algorithm's path

# _vertices = []				# List of int references to vertices: V in the paper
# _oppositeCorners = []		# List of int references to opposite corners: O in the paper
_marked = []				# List of bool indicating whether a vertex has already been visited: M in the paper
_flagged = []				# List of bool indicating whether a triangle has already been visited: U in the paper

_missingTrianglesCount = 0	# The number of triangles not already seen by EdgeBreaker

_halfEdgeId = 0				# Select first half-edge to begin the EdgeBreaker algorithm
_triangleId = 0				# Select the first triangle (always associated with first halfEdge)


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
	global _halfEdgeId, _triangleId, _clers, _deltas

	initData()

	# Try to find a 'C' or an 'S' configuration by rotating the first vertex in the current triangle
	# If not possible, try to find an 'R' or an 'L' configuration with the same method
	for i in range(6):
		if i < 3:	# Search fo 'C' or 'S' configurations
			if getLeftCornerHeId(_halfEdgeId) != -1 and getRightCornerHeId(_halfEdgeId) != -1:
				if isMarked(_halfEdgeId):
					break	# Found an 'S' configuration
				else:
					break	# Found a 'C' configuration
		else:		# Search fo 'R' or 'L' configurations
			if getLeftCornerHeId(_halfEdgeId) != -1:
				break		# Found an 'R' configuration
			if getRightCornerHeId(_halfEdgeId) != -1:
				break		# Found an 'L' configuration
		_halfEdgeId = getNextHeId(_halfEdgeId)

	# First vertex position
	_deltas.append(getVertexPosFromHeId(getPreviousHeId(_halfEdgeId)))
	# Vector from first to second vertex
	_deltas.append(getDistanceVectorFromHeId(getPreviousHeId(_halfEdgeId), _halfEdgeId))
	# Vector from second to third vertex
	_deltas.append(getDistanceVectorFromHeId(_halfEdgeId, getNextHeId(_halfEdgeId)))

	# Mark these vertices as "seen"
	mark(_halfEdgeId)
	mark(getNextHeId(_halfEdgeId))
	mark(getPreviousHeId(_halfEdgeId))


def compress(halfEdgeId):
	global _triangleId, _deltas, _clers

	while True:
		if halfEdgeId == -1:
			return

		print()
		print(f'Current he id: {halfEdgeId}')
		print(f'Current vertex id: {getVertexId(halfEdgeId)}')
		print(f'Marks: {_marked}')
		print(f'Flags: {_flagged}')

		_triangleId = getTriangleFromHeId(halfEdgeId)
		flag(_triangleId)

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

	print("Running MAIN from EdgeBreaker.py")
	mesh = open3d.io.read_triangle_mesh("Models/complex_shape.obj")
	_heMesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)
	
	initCompression()
	compress(_halfEdgeId)

	print(f'CLERS = {_clers}')
	print("Deltas:")
	for v in _deltas:
		print(v)

	open3d.visualization.draw_geometries([_heMesh])
	print(numpy.asarray(_heMesh.triangles))
	print(numpy.asarray(_heMesh.half_edges))

	return 0


if __name__ == '__main__':
	sys.exit(main())
