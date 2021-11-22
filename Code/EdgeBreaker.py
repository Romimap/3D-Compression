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

_heMesh = None 			# The mesh containing half-edges data

_vertices = []			# List of int references to vertices: V in the paper
_oppositeCorners = []	# List of int references to opposite corners: O in the paper
_marked = []			# List of bool indicating whether a vertex has already been visited: M in the paper
_flagged = []			# List of bool indicating whether a triangle has already been visited: U in the paper


# ------------------------------------------------------------
# Data access functions
# ------------------------------------------------------------

## VERTICES

def getVertexId(halfEdgeId):
	return _heMesh.half_edges[halfEdgeId].vertex_indices[0]


def getNextVertexId(halfEdgeId):
	return getVertexId(getNextHeId(halfEdgeId))


def getPreviousVertexId(halfEdgeId):
	return getVertexId(getPreviousHeId(halfEdgeId))


## HALF-EDGES

def getNextHeId(halfEdgeId):
	return _heMesh.half_edges[halfEdgeId].next


def getPreviousHeId(halfEdgeId):
	return getNextHeId(getNextHeId(halfEdgeId))


def getTwinHeId(halfEdgeId):
	return _heMesh.half_edges[halfEdgeId].twin


def getOppositeCornerHEId(halfEdgeId):
	return getPreviousHeId(getTwinHeId(getNextHeId(halfEdgeId)))


## DISTANCE VECTORS

def getDistanceVectorFromVerticesId(fromVertexId, toVertexId):
	return _heMesh.vertices[toVertexId] - _heMesh.vertices[fromVertexId]


def getDistanceVectorFromHeId(fromHeId, toHeId):
	fromVertexId = getVertexId(fromHeId)
	toVertexId = getVertexId(toHeId)
	return getDistanceVectorFromVerticesId(fromVertexId, toVertexId)


## VERTEX POSITION

def getVertexPosFromHeId(halfEdgeId):
	return _heMesh.vertices[getVertexId(halfEdgeId)]


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
	global _vertices, _oppositeCorners, _marked, _flagged

	# Vertices and opposite corners
	halfEdgeCount = len(_heMesh.half_edges)
	for halfEdgeId in range(halfEdgeCount):
		_vertices.append(getVertexId(halfEdgeId))
		_oppositeCorners.append(getOppositeCornerHEId(halfEdgeId))

	# Marks and flags
	_marked = [False] * len(_heMesh.vertices) 	# Marked: if a vertex has been visited or not
	_flagged = [False] * halfEdgeCount			# Flags: if a triangle's corner has been visited or not



# PROCEDURE initCompression (c){
#     GLOBAL M[]={0...}, U[]={0...};              # init tables for marking visited vertices and triangles
#     WRITE(delta, c.p.v.g);                      # store first vertex as a point
#     WRITE(delta, c.v.g – c.p.v.g);              # store second vertex as a difference vector with first
#     WRITE(delta, c.n.v.g – c.v.g);              # store third vertex as a difference vector with second
#     M[c.v] = 1;  M[c.n.v] = 1; M[c.p.v] = 1;    # mark these 3 vertices
#     U[c.t] = 1;                                 # paint the triangle and go to opposite corner
#     Compress (c.o); }                           # start the compression process
def initCompression():
	initData() # GLOBAL M[]={0...}, U[]={0...} + V & O initialization from _heMesh

	deltas = [] # Deltas: first vertex and squence of corrective vertors from current to next vertex

	halfEdgeId = 0 # Select first half-edge to begin the EdgeBreaker algorithm
	# First vertex position
	deltas.append(getVertexPosFromHeId(getPreviousHeId(halfEdgeId))) 					# WRITE(delta, c.p.v.g)
	# Vector from first to second vertex
	deltas.append(getDistanceVectorFromHeId(getPreviousHeId(halfEdgeId), halfEdgeId)) 	# WRITE(delta, c.v.g – c.p.v.g)
	# Vector from second to third vertex
	deltas.append(getDistanceVectorFromHeId(halfEdgeId, getNextHeId(halfEdgeId))) 		# WRITE(delta, c.n.v.g – c.v.g)

	# Mark these vertices
	_marked[_vertices[halfEdgeId]] = True 					# M[c.v] = 1
	_marked[_vertices[getNextHeId(halfEdgeId)]] = True 		# M[c.n.v] = 1
	_marked[_vertices[getPreviousHeId(halfEdgeId)]] = True 	# M[c.p.v] = 1

	# _flagged[] // STOPPED HERE

	print(numpy.asarray(_heMesh.triangles))
	print(numpy.asarray(_heMesh.half_edges))


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():
	global _heMesh
	print("Running MAIN from EdgeBreaker.py")
	mesh = open3d.io.read_triangle_mesh("Models/cube.obj")
	_heMesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)
	initCompression()
	return 0


if __name__ == '__main__':
	sys.exit(main())
