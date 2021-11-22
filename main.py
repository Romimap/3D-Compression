import builtins
import sys
import os
import open3d
import bitarray
import Code
import copy

from Code.Quantization import quantizeVertices
from Code.Quantization import readVerticesBits
from Code.Encryption import scramble
from Code.Encryption import unscramble

import cProfile
from pstats import Stats, SortKey

k = 10

def testFunction():
	print("Testing IO for meshes ...")
	mesh = open3d.io.read_triangle_mesh("Models/suzanne.obj")
	mesh.compute_vertex_normals()
	meshQuantified = copy.deepcopy(mesh)
	meshScrambled = copy.deepcopy(mesh)
	meshUnscrambledOk = copy.deepcopy(mesh)
	meshUnscrambledBad = copy.deepcopy(mesh)

	xoffset = meshQuantified.get_axis_aligned_bounding_box().get_extent()[0] * 1.2
	
	bitstring = quantizeVertices(meshQuantified, k)

	vertices, normals = readVerticesBits(bitstring)
	meshQuantified.vertices = open3d.utility.Vector3dVector(vertices)
	meshQuantified.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshQuantified.translate((1 * xoffset, 0, 0))

	
	bitstringScrambled = scramble(bitstring, k, 'password')
	
	vertices, normals = readVerticesBits(bitstringScrambled)
	meshScrambled.vertices = open3d.utility.Vector3dVector(vertices)
	meshScrambled.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshScrambled.translate((2 * xoffset, 0, 0))

	
	bitstringUnscrambledOk = unscramble(bitstringScrambled, k, 'password')

	vertices, normals = readVerticesBits(bitstringUnscrambledOk)
	meshUnscrambledOk.vertices = open3d.utility.Vector3dVector(vertices)
	meshUnscrambledOk.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshUnscrambledOk.translate((3 * xoffset, 0, 0))


	bitstringUnscrambledBad = unscramble(bitstringScrambled, k, 'badpassword')

	vertices, normals = readVerticesBits(bitstringUnscrambledBad)
	meshUnscrambledBad.vertices = open3d.utility.Vector3dVector(vertices)
	meshUnscrambledBad.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshUnscrambledBad.translate((4 * xoffset, 0, 0))



	#print(bitstring)
	bitArray = bitarray.bitarray([0 for _ in range(8-(len(bitstring)%8))] + list(map(int,bitstring)))
	with open("bits","wb+") as f:
		bitArray.tofile(f)

	open3d.visualization.draw_geometries([mesh, meshQuantified, meshScrambled, meshUnscrambledBad, meshUnscrambledOk])

def main():
	print("Hello world")
	testFunction()
	return 0

if __name__ == '__main__':
	do_profiling = True
	if do_profiling:
		with cProfile.Profile() as pr:
			main()
		with open('profiling_stats.txt', 'w') as stream:
			stats = Stats(pr, stream=stream)
			stats.strip_dirs()
			stats.sort_stats('time')
			stats.dump_stats('.prof_stats')
			stats.print_stats()
	else:
		main()
	sys.exit()
