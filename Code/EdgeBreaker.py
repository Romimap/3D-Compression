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
		return False
	else:
		return _marked[getVertexId(halfEdgeId)]


def flag(halfEdgeId):
	global _flagged

	if halfEdgeId != -1:
		triangleId = _heMesh.half_edges[halfEdgeId].triangle_index
		_flagged[triangleId] = True


def isFlagged(halfEdgeId):
	global _flagged
	
	if halfEdgeId == -1:
		return False
	else:
		triangleId = _heMesh.half_edges[halfEdgeId].triangle_index
		return _flagged[triangleId]


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

def initData():
	global _marked, _flagged, _missingTrianglesCount

	# # Vertices and opposite corners
	# halfEdgeCount = len(_heMesh.half_edges)
	# for halfEdgeId in range(halfEdgeCount):
	# 	_vertices.append(getVertexId(halfEdgeId))
	# 	_oppositeCorners.append(getOppositeCornerHeId(halfEdgeId))

	# Marks and flags
	_marked = [False] * len(_heMesh.vertices) 		# Marks: if a vertex has been visited or not
	_flagged = [False] * len(_heMesh.triangles)		# Flags: if a triangle has been visited or not
	_missingTrianglesCount = len(_heMesh.triangles)	# No triangle has been seen yet

	# Mark boundary vertices as "seen"
	for halfEdge in _heMesh.half_edges:
		if halfEdge.twin == -1:
			_marked[halfEdge.vertex_indices[0]] = True
			_marked[halfEdge.vertex_indices[1]] = True
			
	print("End of initData()")



# PROCEDURE initCompression (c){
#     GLOBAL M[]={0...}, U[]={0...};              # init tables for marking visited vertices and triangles
#     WRITE(delta, c.p.v.g);                      # store first vertex as a point
#     WRITE(delta, c.v.g – c.p.v.g);              # store second vertex as a difference vector with first
#     WRITE(delta, c.n.v.g – c.v.g);              # store third vertex as a difference vector with second
#     M[c.v] = 1;  M[c.n.v] = 1; M[c.p.v] = 1;    # mark these 3 vertices
#     U[c.t] = 1;                                 # paint the triangle and go to opposite corner
#     Compress (c.o); }                           # start the compression process
def initCompression():
	global _halfEdgeId, _triangleId, _clers, _deltas

	initData()	# GLOBAL M[]={0...}, U[]={0...} + V & O initialization from _heMesh

	# Try to find a 'C' or an 'S' configuration by rotating the first vertex in the current triangle
	for i in range(6):
		print(str(i) + " (He id) left: " + str(getLeftCornerHeId(_halfEdgeId)) + ", right: " + str(getRightCornerHeId(_halfEdgeId)))
		print(str(i) + " (Vertex id) left: " + str(getLeftVertexId(_halfEdgeId)) + ", right: " + str(getRightVertexId(_halfEdgeId)))
		if i < 3:
			if getLeftCornerHeId(_halfEdgeId) != -1 and getRightCornerHeId(_halfEdgeId) != -1:
				if isMarked(_halfEdgeId):
					print(str(i) + " found an S configuration")
					break
				else:
					print(str(i) + " found an C configuration")
					break
		else:
			if getLeftCornerHeId(_halfEdgeId) != -1:
				print(str(i) + " found an R configuration")
				break
			if getRightCornerHeId(_halfEdgeId) != -1:
				print(str(i) + " found an L configuration")
				break
		print(str(i) + " turn")
		_halfEdgeId = getNextHeId(_halfEdgeId)
		if i == 5:
			print(str(i) + " found an E configuration")

	# First vertex position
	_deltas.append(getVertexPosFromHeId(getPreviousHeId(_halfEdgeId))) 					# WRITE(delta, c.p.v.g)
	# Vector from first to second vertex
	_deltas.append(getDistanceVectorFromHeId(getPreviousHeId(_halfEdgeId), _halfEdgeId)) # WRITE(delta, c.v.g – c.p.v.g)
	# Vector from second to third vertex
	_deltas.append(getDistanceVectorFromHeId(_halfEdgeId, getNextHeId(_halfEdgeId))) 	# WRITE(delta, c.n.v.g – c.v.g)

	# Mark these vertices as "seen"
	mark(_halfEdgeId)					# M[c.v] = 1
	mark(getNextHeId(_halfEdgeId))		# M[c.n.v] = 1
	mark(getPreviousHeId(_halfEdgeId))	# M[c.p.v] = 1

	# Flag the triangle as "seen"
	flag(_halfEdgeId)	# U[c.t] = 1

	print("End of initCompression()")


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():
	global _heMesh

	print("Running MAIN from EdgeBreaker.py")
	mesh = open3d.io.read_triangle_mesh("Models/triangle.obj")
	_heMesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)
	
	initCompression()

	# open3d.visualization.draw_geometries([_heMesh])
	# print(numpy.asarray(_heMesh.triangles))
	# print(numpy.asarray(_heMesh.half_edges))

	return 0


if __name__ == '__main__':
	sys.exit(main())
