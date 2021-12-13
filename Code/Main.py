import tkinter as tk

import open3d
import sys

from EdgebreakerCompression import compress
from EdgebreakerDecompression import decompress
from Quantization import printBitString, quantizeVertices, quantizedPositionsToBitstring, normalsToBitstring, clersToBitstring, readVerticesBits
from Encryption import scramble, unscramble, xorifyNormals
from ImportExport import objImporter, objExporter, writeFile, readFile


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

def cryptoCompress (password, model, filename):
	k = 10
	#Load, Quantize and Process our mesh
	originalMesh = objImporter(f'Models/{model}')
	quantizeVertices(originalMesh, k)
	mesh = preProcess(model, originalMesh)
	bitstring = quantizeVertices(originalMesh, k) #a bit ugly but we need to call this on the preprocessed mesh to get a valid header for the bitstring

	#Run EdgeBreaker
	clers, deltas, normals = compress(mesh, debug=False)

	#Add our deltas, normals and clers to our bitsting
	verticesBitstring = quantizedPositionsToBitstring(deltas, k)
	verticesBitstring = verticesBitstring.replace('-', '1') #NOTE: negative numbers can now occur, decompression should take this into account
	normals = mesh.vertex_normals #NOTE: Placeholder normal array
	normalsBitstring = normalsToBitstring(normals, k)
	clersBitstring = clersToBitstring(clers)

	bitstring += verticesBitstring
	bitstring += normalsBitstring
	bitstring += clersBitstring

	#From our bitstring, scramble positions and normals
	bitstring = scramble(bitstring, 10, password)
	bitstring = xorifyNormals(bitstring, 10, password)

	printBitString(bitstring)

	print(str(len(deltas)) + " " + str(len(normals)) + " " + str(len(clers)))
	writeFile(bitstring, filename)
	#open3d.visualization.draw_geometries([mesh])

def cryptoExtract (password, filename):
	bitstring = readFile(filename)
	printBitString(bitstring)

	#Decrypt our bitstring
	bitstring = xorifyNormals(bitstring, 10, password)
	bitstring = unscramble(bitstring, 10, password)

	#Read the bitsting and extract data
	deltas, normals, clers = readVerticesBits(bitstring)

	print(str(len(deltas)) + " " + str(len(normals)) + " " + str(len(clers)))

	#Run the Edgebreaker decryption
	decompressedMesh = decompress(clers, deltas, normals, False)

	open3d.visualization.draw_geometries([decompressedMesh])

	return decompressedMesh



def main():

	print(f'\n\n\n\n\nRunning MAIN from EdgeBreakerDemo.py')

	doCompress = True
	doPreProcess = True
	model = "Sphere.obj"
	filename = "Sphere.rfcp"

	#Crypto Compress our mesh
	cryptoCompress("passwd", model, filename)

	#Crypto Extract our mesh
	cryptoExtract("passwd", filename)

	exit()


if __name__ == '__main__':
	sys.exit(main())


