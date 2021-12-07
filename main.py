import builtins
import sys
import os
import numpy
import open3d
import bitarray
import time

import Code
import copy
import math

from Code.Quantization import quantizeVertices, quantizedPositionsToBitstring, normalsToBitstring, printBitString, simplify
from Code.Quantization import readVerticesBits
from Code.Encryption import scramble, xorifyNormals
from Code.Encryption import unscramble
from Code.Huffman import makeCodebook

import cProfile
from pstats import Stats, SortKey

k = 10
MODELNAME = "bunny_normals"

def interpolate (A, B, C):
	n = A + B + C
	l = numpy.sqrt(pow(n[0], 2) + pow(n[1], 2) + pow(n[2], 2))
	return n / l


def objImporter(filepath):
	vertexIdDict = {} #Used to check if a vertex already exists
	
	#Positions & normals from the .obj
	positions = []
	normals = []

	#Vertices & Vertex Normals from the mesh
	vertices = []
	vertexNormals = []
	triangles = []
	triangleNormals = []

	with open(filepath) as file:
		for line in file:
			line = line.rstrip()
			value = line.split(" ")
			if value[0] == "v":
				x, y, z = float(value[1]), float(value[2]), float(value[3])
				positions.append(numpy.array([x, y, z]))
			if value[0] == "vn":
				x, y, z = float(value[1]), float(value[2]), float(value[3])
				normals.append(numpy.array([x, y, z]))
			if value[0] == "f":
				if value[1] not in vertexIdDict:
					id = len(vertices)
					vid = int(value[1].split("/")[0]) - 1
					nid = int(value[1].split("/")[2]) - 1
					vertices.append(positions[vid])
					vertexNormals.append(normals[nid])
					vertexIdDict[value[1]] = id
				A = vertexIdDict[value[1]]

				if value[2] not in vertexIdDict:
					id = len(vertices)
					vid = int(value[2].split("/")[0]) - 1
					nid = int(value[2].split("/")[2]) - 1
					vertices.append(positions[vid])
					vertexNormals.append(normals[nid])
					vertexIdDict[value[2]] = id
				B = vertexIdDict[value[2]]
				if value[3] not in vertexIdDict:
					id = len(vertices)
					vid = int(value[3].split("/")[0]) - 1
					nid = int(value[3].split("/")[2]) - 1
					vertices.append(positions[vid])
					vertexNormals.append(normals[nid])
					vertexIdDict[value[3]] = id
				C = vertexIdDict[value[3]]
				triangles.append(numpy.array([A, B, C]))
				triangleNormals.append(interpolate(vertexNormals[A], vertexNormals[B], vertexNormals[C]))

	mesh = open3d.geometry.TriangleMesh.create_sphere()
	mesh.vertices = open3d.utility.Vector3dVector(vertices)
	mesh.vertex_normals = open3d.utility.Vector3dVector(vertexNormals)
	mesh.triangles = open3d.utility.Vector3iVector(triangles)
	mesh.compute_triangle_normals()
	print(numpy.asarray(mesh.triangle_normals))
	print(triangleNormals)
	print(numpy.asarray(open3d.utility.Vector3dVector(triangleNormals)))
	mesh.triangle_normals = open3d.utility.Vector3dVector(triangleNormals)


	print(numpy.asarray(mesh.vertices))
	print(numpy.asarray(mesh.vertex_normals))
	print(numpy.asarray(mesh.triangles))

	#open3d.visualization.draw_geometries([mesh])

	return mesh
				
def objExporter(filepath, mesh):
	os.system('rm ' + filepath)
	file = open(filepath, "a")
	for v in mesh.vertices:
		file.write("v " + "{:.4f}".format(v[0]) + " " + "{:.4f}".format(v[1]) + " " + "{:.4f}".format(v[2]) + "\n")
	for n in mesh.vertex_normals:
		file.write("vn " + "{:.4f}".format(n[0]) + " " + "{:.4f}".format(n[1]) + " " + "{:.4f}".format(n[2]) + "\n")
	for f in mesh.triangles:
		file.write("f " + str(f[0]+1) + "//" + str(f[0]+1) + " " + str(f[1]+1) + "//" + str(f[1]+1) + " " + str(f[2]+1) + "//" + str(f[2]+1) + "\n")


def testAnimation():
	mesh = open3d.io.read_triangle_mesh("Models/"+ MODELNAME + ".obj")
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
	#mesh = open3d.io.read_triangle_mesh("Models/suzanne.obj")
	mesh = objImporter("Models/"+ MODELNAME + ".obj")
	#mesh.compute_vertex_normals()
	#mesh.compute_triangle_normals()
	meshQuantified = copy.deepcopy(mesh)
	meshScrambled = copy.deepcopy(mesh)
	meshUnscrambledOk = copy.deepcopy(mesh)
	meshUnscrambledBad = copy.deepcopy(mesh)
	meshXorified = copy.deepcopy(mesh)
	meshUnxorifiedOk = copy.deepcopy(mesh)
	meshUnxorifiedBad = copy.deepcopy(mesh)

	xoffset = meshQuantified.get_axis_aligned_bounding_box().get_extent()[0] * 1.2
	yoffset = meshQuantified.get_axis_aligned_bounding_box().get_extent()[0] * 1.2

	
	print("DEFAULT")

	bitstring = quantizeVertices(meshQuantified, k)

	vertices, normals = readVerticesBits(bitstring)

	meshQuantified.vertices = open3d.utility.Vector3dVector(vertices)
	meshQuantified.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshQuantified.translate((1 * xoffset, 0, 0))

	print("\nSCRAMBLED\n")

	bitstringScrambled = scramble(bitstring, k, 'password')
	
	vertices, normals = readVerticesBits(bitstringScrambled)
	meshScrambled.vertices = open3d.utility.Vector3dVector(vertices)
	meshScrambled.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshScrambled.translate((0 * xoffset, -1 * yoffset, 0))

	objExporter ("scrambled.obj", meshScrambled)


	print("\nUNSCRAMBLED OK\n")
	
	bitstringUnscrambledOk = unscramble(bitstringScrambled, k, 'password')

	vertices, normals = readVerticesBits(bitstringUnscrambledOk)
	meshUnscrambledOk.vertices = open3d.utility.Vector3dVector(vertices)
	meshUnscrambledOk.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshUnscrambledOk.translate((1 * xoffset, -1 * yoffset, 0))

	objExporter ("unscrambled.obj", meshUnscrambledOk)


	print("\nUNSCRAMBLED BAD\n")

	bitstringUnscrambledBad = unscramble(bitstringScrambled, k, 'badpassword')

	vertices, normals = readVerticesBits(bitstringUnscrambledBad)
	meshUnscrambledBad.vertices = open3d.utility.Vector3dVector(vertices)
	meshUnscrambledBad.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshUnscrambledBad.translate((2 * xoffset, -1 * yoffset, 0))

	print("\nXORIFY\n")

	bitstringXorified = xorifyNormals(bitstring, k, 'password')

	vertices, normals = readVerticesBits(bitstringXorified)
	meshXorified.vertices = open3d.utility.Vector3dVector(vertices)
	meshXorified.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshXorified.translate((0 * xoffset, -2 * yoffset, 0))

	objExporter ("xorified.obj", meshXorified)


	print("\nUNXORIFY OK\n")

	bitstringUnorifiedOk = xorifyNormals(bitstringXorified, k, 'password')

	vertices, normals = readVerticesBits(bitstringUnorifiedOk)
	meshUnxorifiedOk.vertices = open3d.utility.Vector3dVector(vertices)
	meshUnxorifiedOk.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshUnxorifiedOk.translate((1 * xoffset, -2 * yoffset , 0))

	objExporter ("unxorified.obj", meshUnxorifiedOk)

	print("\nUNXORIFY BAD\n")

	bitstringUnorifiedBad = xorifyNormals(bitstringXorified, k, 'badpassword')

	vertices, normals = readVerticesBits(bitstringUnorifiedBad)
	meshUnxorifiedBad.vertices = open3d.utility.Vector3dVector(vertices)
	meshUnxorifiedBad.vertex_normals = open3d.utility.Vector3dVector(normals)
	meshUnxorifiedBad.translate((2 * xoffset, -2 * yoffset, 0))

	bitArray = bitarray.bitarray([0 for _ in range(8-(len(bitstring)%8))] + list(map(int,bitstring)))
	with open("bits","wb+") as f:
		bitArray.tofile(f)

	open3d.visualization.draw_geometries([mesh, meshQuantified, meshScrambled, meshUnscrambledBad, meshUnscrambledOk, meshXorified, meshUnxorifiedOk, meshUnxorifiedBad])


#FOR A FILENAME, RETURNS A BITSTRING THAT CAN BE USED TO WRITE A FILE
def cryptoCompress(password, filename):
	mesh = objImporter(filename)
	bitstring = quantizeVertices(mesh, k)
	verticesBitstring = quantizedPositionsToBitstring(numpy.asarray(mesh.vertices), k)
	normalsBitstring = normalsToBitstring(numpy.asarray(mesh.vertex_normals), k)

	bitstring += verticesBitstring
	bitstring += normalsBitstring

	bitstring = scramble(bitstring, 10, password)
	bitstring = xorifyNormals(bitstring, 10, password)
	
	return bitstring

#FOR A BITSTRING, RETURNS A MESH
def cryptoExcract(password, bitstring):
	mesh = objImporter("./Models/" + MODELNAME + ".obj") #NOTE: remove that
	bitstring = xorifyNormals(bitstring, 10, password)
	bitstring = unscramble(bitstring, 10, password)
	vertices, normals = readVerticesBits(bitstring)
	mesh.vertices = open3d.utility.Vector3dVector(vertices)
	mesh.vertex_normals = open3d.utility.Vector3dVector(normals)
	return mesh

#WRITES A FILE FROM A BITSTRING
def writeFile(bitstring, filename):
	bitArray = bitarray.bitarray([0 for _ in range(8-(len(bitstring)%8))] + list(map(int,bitstring)))
	with open(filename,"wb+") as f:
		bitArray.tofile(f)

def readFile(filename):
	bitstring = ''
	with open(filename, "rb") as f:
		bytes = (list(map(str,numpy.fromfile(f,"u1"))))
		for i in bytes:
			bitstring += '{0:08b}'.format(int(i))

	bitstring = bitstring[8:]
	return bitstring

def main():
	#testFunction()
		
	#bitstring = cryptoCompress('password', './Models/' + MODELNAME + '.obj')
	#printBitString(bitstring)
	#writeFile(bitstring, MODELNAME + ".rfcp")
	#bitstring = readFile(MODELNAME + ".rfcp")
	#printBitString(bitstring)
	#mesh = cryptoExcract('password', bitstring)

	mesh = open3d.io.read_triangle_mesh("Models/" + MODELNAME + ".obj")
	quantizeVertices(mesh, 10)
	simplify(mesh)
	mesh.compute_vertex_normals()
	mesh.compute_triangle_normals()
	open3d.visualization.draw_geometries([mesh])
	heMesh = open3d.geometry.HalfEdgeTriangleMesh.create_from_triangle_mesh(mesh)
	print("done")


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
