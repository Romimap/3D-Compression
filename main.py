import sys
import os
import open3d
import bitarray

import Code
from Code.Quantization import quantizeVertices


def testFunction():
	print("Testing IO for meshes ...")
	mesh = open3d.io.read_triangle_mesh("Models/bunny.obj")
	mesh.vertices, bitstring = quantizeVertices(mesh.vertices, 3)

	mesh.compute_vertex_normals()
	open3d.visualization.draw_geometries([mesh])

	bitArray = bitarray.bitarray([0 for _ in range(8-(len(bitstring)%8))] + list(map(int,bitstring)))

	with open("bits","wb+") as f:
		bitArray.tofile(f)


def main():
	print("Hello world")
	
	testFunction()

	return 0

if __name__ == '__main__':
	sys.exit(main())
