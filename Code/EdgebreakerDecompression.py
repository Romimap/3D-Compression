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


## EDGEBREAKER DECOMPRESSION SPECIFIC

_V = []
_O = []
_G = []

_T = 0
_N = 2

_deltasIndex = 0
_clersIndex = 0


# ------------------------------------------------------------
# Data access functions
# ------------------------------------------------------------

## HALF-EDGES

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


def right(c):
	return _O[next(c)]


def left(c):
	return _O[previous(c)]


## TRIANGLES

def triangle(c):
	return int(c / 3)


# ------------------------------------------------------------
# CALCULATIONS
# ------------------------------------------------------------

## VECTORS

def addVectors3D(v1, v2):
	return [v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]]


# ------------------------------------------------------------
# DELTAS
# ------------------------------------------------------------

def readDeltas():
	global _deltasIndex

	delta = _deltas[_deltasIndex]
	_deltasIndex += 1
	return delta


# ------------------------------------------------------------
# CLERS
# ------------------------------------------------------------

def readClers():
	global _clersIndex

	letter = _clers[_clersIndex]
	_clersIndex += 1
	return letter


# ------------------------------------------------------------
# FLAGS
# ------------------------------------------------------------

## MARKS

def mark(c):
	global _marked

	if c != -1:
		_marked[_V(c)] = True


def isMarked(c):
	global _marked

	if c == -1:
		return True
	else:
		return _marked[_V(c)]


## FLAGS

def flag(c):
	global _flagged

	if c != -1:
		_flagged[triangle(c)] = True


def isFlagged(c):
	global _flagged
	
	if c == -1:
		return True
	else:
		return _flagged[triangle(c)]


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
# Edgebreaker decompression part
# ------------------------------------------------------------

def decompressConnectivity(c):
	global _V, _O, _T, _N

	while True:
		_T += 1
		_O[c], _O[3 * _T] = 3 * _T, c
		_V[3 * _T + 1], _V[3 * _T + 2] = _V[previous(c)], _V[next(c)]
		c = next(_O[c])

		letter = readClers()
		cn = next(c)

		if letter == 'C':
			_O[cn] = -1
			_N += 1
			_V[3 * _T] = _N

		elif letter == 'L':
			_O[cn] = -2
			zip(cn)
			
		elif letter == 'R':
			_O[c] = -2
			c = cn

		elif letter == 'S':
			decompressConnectivity(c)
			c = cn

		elif letter == 'E':
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
