import sys
import os
import open3d
import bitarray
import Code

from Code.Quantization import quantizeVertices
from Code.Quantization import readVerticesBits

def testFunction():
	print("Testing IO for meshes ...")
	mesh = open3d.io.read_triangle_mesh("Models/bunny.obj")
	mesh.compute_vertex_normals()
	bitstring = quantizeVertices(mesh, 4)

	#print(bitstring)
	bitArray = bitarray.bitarray([0 for _ in range(8-(len(bitstring)%8))] + list(map(int,bitstring)))

	with open("bits","wb+") as f:
		bitArray.tofile(f)

	readVerticesBits(bitstring)

	open3d.visualization.draw_geometries([mesh])



def main():
	print("Hello world")
	testFunction()
	return 0

if __name__ == '__main__':
	sys.exit(main())
