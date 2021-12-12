
import open3d
import sys

from EdgebreakerCompression import compress
from EdgebreakerDecompression import decompress


def showMesh(mesh):
	mesh.paint_uniform_color([0.6, 0.6, 0.6])
	open3d.visualization.draw_geometries([mesh])


def main():
	print(f'\n\n\n\n\nRunning MAIN from EdgeBreakerDemo.py')

	originalMesh = open3d.io.read_triangle_mesh("Models/sphere.obj")
	clers, deltas, normals = compress(originalMesh, False)

	decompressedMesh = decompress(clers, deltas, normals, False)
	showMesh(decompressedMesh)

	return 0


if __name__ == '__main__':
	sys.exit(main())
