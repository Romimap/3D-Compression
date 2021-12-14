
import math
import numpy
import open3d
import sys

from EdgebreakerCompression import compress
from EdgebreakerDecompression import decompress
from MeshQualityEvaluation import evaluateWithHausdorff


def showMesh(mesh):
	mesh.paint_uniform_color([0.6, 0.6, 0.6])
	open3d.visualization.draw_geometries([mesh])


def calculateTriangleNormals(mesh):
	triangles = mesh.triangles
	vertexNormals = mesh.vertex_normals
	triangleNormals = numpy.empty((len(triangles), 3))

	i = 0
	for t in triangles:
		n1 = vertexNormals[t[0]]
		n2 = vertexNormals[t[1]]
		n3 = vertexNormals[t[2]]
		triangleNormal = [n1[0] + n2[0] + n3[0], n1[1] + n2[1] + n3[1], n1[2] + n2[2] + n3[2]]
		length = math.sqrt(math.pow(triangleNormal[0], 2) + math.pow(triangleNormal[1], 2) + math.pow(triangleNormal[1], 2))
		triangleNormal[0] /= length
		triangleNormal[1] /= length
		triangleNormal[2] /= length

		triangleNormals[i] = triangleNormal
		i += 1

	mesh.vertex_normals = open3d.utility.Vector3dVector(vertexNormals)
	mesh.triangle_normals = open3d.utility.Vector3dVector(triangleNormals)

	return mesh


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
	model = "Sphere.obj"

	# Read original mesh and print stats
	mesh = open3d.io.read_triangle_mesh(f'Models/{model}')

	print(f'{model} model stats:')
	print(f'Vertices: {len(mesh.vertices)}')
	print(f'Triangles: {len(mesh.triangles)}')
	
	# Verify if normals are specified, if not, calculate them
	if mesh.has_vertex_normals():
		mesh = calculateTriangleNormals(mesh)
	else:
		mesh.compute_vertex_normals()
		mesh.compute_triangle_normals()
	
	# Pre-process the mesh if specified
	if doPreProcess:
		mesh = preProcess(model, mesh)

	if not doCompress:		# Show original or pre-processed mesh
		showMesh(mesh)
	else:					# Compress original or pre-processed mesh, decompress compressed mesh, and show decompressed mesh
		try:
			# Compress the mesh
			clers, deltas, normals = compress(mesh, False)
		except:
			print(f'{model} is not suitable to Edgebreaker. Use a simple compression.')

		# Decompress and show the mesh
		decompressedMesh = decompress(clers, deltas, normals, False)

		# Evaluate decompressed mesh quality
		# originalMesh = open3d.io.read_triangle_mesh(f'Models/{model}')
		# evaluateWithHausdorff(originalMesh, decompressedMesh)

		# Show the mesh
		showMesh(decompressedMesh)

	return 0


if __name__ == '__main__':
	sys.exit(main())
