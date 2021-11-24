import sys
import os
import open3d
import bitarray
import time

import Code

from Code.Quantization import quantizeVertices
from Code.Quantization import readVerticesBits


def testAnimation():
	mesh = open3d.io.read_triangle_mesh("Models/bunny.obj")
	mesh.compute_vertex_normals()
	mesh.paint_uniform_color([0.8, 0.8, 0.8])

	heMesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)

	boundaryVertices = []
	for halfEdge in heMesh.half_edges:
		if halfEdge.twin == -1:
			boundaryVertices.append(halfEdge.vertex_indices[0])

	vis = open3d.visualization.Visualizer()
	vis.create_window()

	# geometry is the point cloud used in your animaiton
	vis.add_geometry(heMesh)

	lastUpdateTime = time.time()
	delay = 0.5
	for bv in boundaryVertices:
		while True:
			if lastUpdateTime + delay <= time.time():
				lastUpdateTime += delay
				heMesh.vertex_colors[bv] = [0.8, 0, 0]
				vis.update_geometry(heMesh)
				print(f'Update at: {lastUpdateTime}')
				break

			vis.poll_events()
			vis.update_renderer()

	vis.run()


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
