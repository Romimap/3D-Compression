
import open3d
import sys

from EdgebreakerCompression import compress
from EdgebreakerDecompression import decompress


def showMesh(mesh):
	mesh.paint_uniform_color([0.6, 0.6, 0.6])
	open3d.visualization.draw_geometries([mesh])


def preProcess(model, mesh):
	mesh = mesh.remove_degenerate_triangles()
	print(f'After remove_degenerate_triangles()... {model} model stats:')
	print(f'Vertices: {len(mesh.vertices)}')
	print(f'Triangles: {len(mesh.triangles)}')

	mesh = mesh.remove_duplicated_triangles()
	print(f'After remove_duplicated_triangles()... {model} model stats:')
	print(f'Vertices: {len(mesh.vertices)}')
	print(f'Triangles: {len(mesh.triangles)}')

	mesh = mesh.remove_duplicated_vertices()
	print(f'After remove_duplicated_vertices()... {model} model stats:')
	print(f'Vertices: {len(mesh.vertices)}')
	print(f'Triangles: {len(mesh.triangles)}')

	mesh = mesh.remove_non_manifold_edges()
	print(f'After remove_non_manifold_edges()... {model} model stats:')
	print(f'Vertices: {len(mesh.vertices)}')
	print(f'Triangles: {len(mesh.triangles)}')

	mesh = mesh.remove_unreferenced_vertices()
	print(f'After remove_unreferenced_vertices()... {model} model stats:')
	print(f'Vertices: {len(mesh.vertices)}')
	print(f'Triangles: {len(mesh.triangles)}')

	return mesh


def main():
	print(f'\n\n\n\n\nRunning MAIN from EdgeBreakerDemo.py')

	doCompress = True
	doPreProcess = True
	model = "XYZ Dragon.obj"

	# Read original mesh and print stats
	originalMesh = open3d.io.read_triangle_mesh(f'Models/{model}')

	print(f'{model} model stats:')
	print(f'Vertices: {len(originalMesh.vertices)}')
	print(f'Triangles: {len(originalMesh.triangles)}')
	
	# Pre-process the mesh if specified
	mesh = originalMesh
	if doPreProcess:
		mesh = preProcess(model, originalMesh)

	if not doCompress:		# Show original or pre-processed mesh
		mesh.compute_vertex_normals()
		mesh.compute_triangle_normals()
		showMesh(mesh)
	else:					# Compress original or pre-processed mesh, decompress compressed mesh, and show decompressed mesh
		try:
			# Compress the mesh
			clers, deltas, normals = compress(mesh, False)

			# Decompress and show the mesh
			decompressedMesh = decompress(clers, deltas, normals, False)
			showMesh(decompressedMesh)
		except:
			print(f'{model} is not suitable to Edgebreaker. Use a simple compression.')

	return 0


if __name__ == '__main__':
	sys.exit(main())
