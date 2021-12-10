
import open3d
import sys

from EdgebreakerCompression import compress
from EdgebreakerDecompression import decompress


def main():
	print(f'\n\n\n\n\nRunning MAIN from EdgeBreakerDemo.py')

	mesh = open3d.io.read_triangle_mesh("Models/Sphere.obj")
	clers, deltas, normals = compress(mesh, True)

	mesh = decompress(clers, deltas, normals, True)

	return 0


if __name__ == '__main__':
	sys.exit(main())
