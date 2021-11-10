import sys
import os
import open3d


def testFunction():
	print("Testing IO for meshes ...")
	mesh = open3d.io.read_triangle_mesh("Models/bunny.obj")
	print(mesh)


def main():
	print("Hello world")
	
	testFunction()

	return 0


if __name__ == '__main__':
	sys.exit(main())
