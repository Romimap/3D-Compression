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

from datetime import datetime
from pstats import Stats


# ------------------------------------------------------------
# Global variables
# ------------------------------------------------------------

## DEBUG AND VISUALIZATION

_doProfiling = False				# Do some profiling

_debug = False						# True if you want to enable color changes and delay between draw calls
_debugPrint = False					# True if you want to enable debug prints

_debugDelayPerFrame = 0.01			# Delay between draw calls

_visualizer = None					# The visualizer (the window)
_lastUpdateTime = None				# Last visual update time in seconds


## EDGEBREAKER RELATED

_heMesh = None 		# The mesh containing half-edges data

_clers = None		# String storing the CLERS steps of the EdgeBreaker algorithm's path
_deltas = None		# List of 3D points/vectors storing the first points and the correction vectors
_normals = None		# TODO

_M = None			# List of bool indicating whether a vertex has already been visited
_U = None			# List of bool indicating whether a triangle has already been visited


## EDGEBREAKER DECOMPRESSION SPECIFIC

_V = None			# Vertices id of each corner
_O = None			# Opposite corner id of each corner
_G = None			# Geometry (position) of each vertex

_T = None			# Current triangle id
_N = None			# Current vertex id

_deltasIndex = None
_clersIndex = None


def initVars():
	global _visualizer, _lastUpdateTime
	global _heMesh, _clers, _deltas, _normals, _M, _U
	global _V, _O, _G, _T, _N, _deltasIndex, _clersIndex
	
	## DEBUG AND VISUALIZATION

	_visualizer = None		# The visualizer (the window)
	_lastUpdateTime = -1	# Last visual update time in seconds

	## EDGEBREAKER RELATED

	_heMesh = None 			# The mesh containing half-edges data

	_clers = ""				# String storing the CLERS steps of the EdgeBreaker algorithm's path
	_deltas = []			# List of 3D points/vectors storing the first points and the correction vectors
	_normals = []			# TODO

	_M = []					# List of bool indicating whether a vertex has already been visited
	_U = []					# List of bool indicating whether a triangle has already been visited

	## EDGEBREAKER DECOMPRESSION SPECIFIC

	_V = []					# Vertices id of each corner
	_O = []					# Opposite corner id of each corner
	_G = []					# Geometry (position) of each vertex

	_T = 0					# Current triangle id
	_N = 2					# Current vertex id

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

def oppositeVector(v):
	return [-v[0], -v[1], -v[2]]


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


def debugPrintInfos():
	if not _debugPrint:
		return
	
	debugPrint(f'\n##########   DEBUG   ##########')
	# debugPrint(f'? Vertices geometry:')
	# for i in range(len(_G)):
	# 	debugPrint(f'{i} = {_G[i]}')

	# debugPrint(f'? Corners → Vertices id:')
	# for i in range(len(_V)):
	# 	debugPrint(f'{i} → {_V[i]}')

	debugPrint(f'? nbTriangles = {(len(_V) / 3)}/{(1 + len(_clers))}')
	debugPrint(f'#######  END OF DEBUG   #######\n')


# ------------------------------------------------------------
# Edgebreaker decompression part
# ------------------------------------------------------------

def zipCorner(c):
	global _V, _O, _G, _T, _N, _M, _U, zipCount

	b = next(c)

	while _O[b] >= 0:
		b = next(_O[b])
	
	if _O[b] != -1:
		return None
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
		return c
	else:
		return None


def zip(c):
	while True:
		c = zipCorner(c)
		if c == None:
			return


def recreateMesh():
	triangles = []
	triangle = []

	i = 0
	for vertexId in _V:
		i += 1
		triangle.append(vertexId)
		if i == 3:
			i = 0
			triangles.append(triangle)
			triangle = []

	vertices = open3d.utility.Vector3dVector(_G)
	triangles = open3d.utility.Vector3iVector(triangles)

	mesh = open3d.geometry.TriangleMesh(vertices, triangles)
	mesh.compute_vertex_normals()
	mesh.compute_triangle_normals()

	return mesh


def decompressConnectivity(c):
	global _V, _O, _G, _T, _N, _M, _U

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


def decompressVertices(c):
	global _V, _O, _G, _T, _N, _M, _U

	while True:
		_U[triangle(c)] = True
		if _M[_V[c]] == False:
			_N += 1
			_G[_N] = addVectors3D(addVectors3D(addVectors3D(_G[_V[previous(c)]], _G[_V[next(c)]]), oppositeVector(_G[_V[_O[c]]])), readDeltas())
			_M[_V[c]] = True
			c = right(c)
		else:
			if _U[triangle(right(c))] == True:
				if _U[triangle(left(c))] == True:
					return
				else:
					c = left(c)
			else:
				if _U[triangle(left(c))] == True:
					c = right(c)
				else:
					decompressVertices(right(c))
					c = left(c)



def initDecompression():
	global _V, _O, _G, _T, _N, _M, _U

	# Initialize arrays
	verticesCount = 3 + _clers.count('C')
	trianglesCount = 1 + len(_clers)
	halfEdgesCount = 3 * trianglesCount

	debugPrint(f'_vertices: {verticesCount}')
	debugPrint(f'_triangles: {trianglesCount}')
	debugPrint(f'_halfEdges: {halfEdgesCount}')

	_V = [0] * halfEdgesCount
	_V[1], _V[2] = 1, 2

	_O = [-3] * halfEdgesCount
	_O[0], _O[2] = -1, -1

	_T = 0
	_N = 2

	decompressConnectivity(1)

	_M = [False] * verticesCount
	_U = [False] * trianglesCount

	_G = [[0, 0, 0]] * verticesCount
	_G[0] = readDeltas()
	_G[1] = addVectors3D(_G[0], readDeltas())
	_G[2] = addVectors3D(_G[1], readDeltas())
	_N = 2

	_M[0] = True
	_M[1] = True
	_M[2] = True
	_U[0] = True

	decompressVertices(_O[1])


# ------------------------------------------------------------
# ONLY "PUBLIC" FUNCTION (TO IMPORT)
# ------------------------------------------------------------

def decompress(clers, deltas, normals, debug = False):
	global _clers, _deltas, _normals, _debugPrint

	_debugPrint = debug

	print(f'Edgebreaker decompression starting at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')

	initVars()
	
	debugInit()

	_clers = clers
	_deltas = deltas
	_normals = normals

	initDecompression()

	mesh = recreateMesh()

	debugPrintInfos()

	debugEnd()

	print(f'Edgebreaker decompression ending at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')

	return mesh


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
	global _debug

	_debug = False
	doDebugPrint = True

	print(f'\n\n\n\n\nRunning MAIN from EdgeBreakerDecompression.py')
	
	mesh = decompress(None, None, None, doDebugPrint)

	return 0


if __name__ == '__main__':
	if _doProfiling:
		sys.exit(doProfiling())
	else:
		sys.exit(main())
