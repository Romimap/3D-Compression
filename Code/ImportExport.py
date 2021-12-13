import os
import numpy
import open3d
import bitarray

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


#Writes and Read from and to a bitstring
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

	bitstring = bitstring[7:]
	return bitstring
