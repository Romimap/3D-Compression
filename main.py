import builtins
import sys
import os
import open3d
import bitarray
import Code
import copy

from Code.Quantization import quantizeVertices
from Code.Quantization import readVerticesBits
from Code.Encryption import scramble, xorifyNormals
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
	meshXorified = copy.deepcopy(mesh)
	meshUnxorifiedOk = copy.deepcopy(mesh)
	meshUnxorifiedBad = copy.deepcopy(mesh)

	xoffset = meshQuantified.get_axis_aligned_bounding_box().get_extent()[0] * 1.2
	
	print("DEFAULT")

	bitstring = quantizeVertices(meshQuantified, k)

	vertices, normals = readVerticesBits(bitstring)
	meshQuantified.vertices = open3d.utility.Vector3dVector(vertices)
	meshQuantified.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshQuantified.compute_vertex_normals()
	meshQuantified.translate((1 * xoffset, 0, 0))

	open3d.io.write_triangle_mesh ("default.obj", meshQuantified)

	print("\nSCRAMBLED\n")

	bitstringScrambled = scramble(bitstring, k, 'password')
	
	vertices, normals = readVerticesBits(bitstringScrambled)
	meshScrambled.vertices = open3d.utility.Vector3dVector(vertices)
	meshScrambled.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshScrambled.compute_vertex_normals()
	meshScrambled.translate((2 * xoffset, 0, 0))

	open3d.io.write_triangle_mesh ("scrambled.obj", meshScrambled)


	print("\nUNSCRAMBLED OK\n")
	
	bitstringUnscrambledOk = unscramble(bitstringScrambled, k, 'password')

	vertices, normals = readVerticesBits(bitstringUnscrambledOk)
	meshUnscrambledOk.vertices = open3d.utility.Vector3dVector(vertices)
	meshUnscrambledOk.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshUnscrambledOk.compute_vertex_normals()
	meshUnscrambledOk.translate((3 * xoffset, 0, 0))

	open3d.io.write_triangle_mesh ("unscrambled.obj", meshUnscrambledOk)


	print("\nUNSCRAMBLED BAD\n")

	bitstringUnscrambledBad = unscramble(bitstringScrambled, k, 'badpassword')

	vertices, normals = readVerticesBits(bitstringUnscrambledBad)
	meshUnscrambledBad.vertices = open3d.utility.Vector3dVector(vertices)
	meshUnscrambledBad.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshUnscrambledBad.compute_vertex_normals()
	meshUnscrambledBad.translate((4 * xoffset, 0, 0))

	print("\nXORIFY\n")

	bitstringXorified = xorifyNormals(bitstring, k, 'password')

	vertices, normals = readVerticesBits(bitstringXorified)
	meshXorified.vertices = open3d.utility.Vector3dVector(vertices)
	meshXorified.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshXorified.compute_vertex_normals()
	meshXorified.translate((5 * xoffset, 0, 0))

	open3d.io.write_triangle_mesh ("xorified.obj", meshXorified)


	print("\nUNXORIFY OK\n")

	bitstringUnorifiedOk = xorifyNormals(bitstringXorified, k, 'password')

	vertices, normals = readVerticesBits(bitstringUnorifiedOk)
	meshUnxorifiedOk.vertices = open3d.utility.Vector3dVector(vertices)
	meshUnxorifiedOk.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshUnxorifiedOk.compute_vertex_normals()
	meshUnxorifiedOk.translate((6 * xoffset, 0, 0))

	open3d.io.write_triangle_mesh ("unxorified.obj", meshUnxorifiedOk)

	print("\nUNXORIFY BAD\n")

	bitstringUnorifiedBad = xorifyNormals(bitstringXorified, k, 'badpassword')

	vertices, normals = readVerticesBits(bitstringUnorifiedBad)
	meshUnxorifiedBad.vertices = open3d.utility.Vector3dVector(vertices)
	meshUnxorifiedBad.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshUnxorifiedBad.compute_vertex_normals()
	meshUnxorifiedBad.translate((7 * xoffset, 0, 0))

	bitArray = bitarray.bitarray([0 for _ in range(8-(len(bitstring)%8))] + list(map(int,bitstring)))
	with open("bits","wb+") as f:
		bitArray.tofile(f)

	open3d.visualization.draw_geometries([mesh, meshQuantified, meshScrambled, meshUnscrambledBad, meshUnscrambledOk, meshXorified, meshUnxorifiedOk, meshUnxorifiedBad])


def main():
	print("Hello world")
	os.system("rm -rf ./out")
	testFunction()
	os.system("mv *.obj ./out")
	os.system("mv *.mtl ./out")
	
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
