from os import stat
import tkinter as tk
import traceback
import copy
import numpy

from tkinter.ttk import Progressbar
from tkinter import ttk
from tkinter import filedialog
from tkinter import scrolledtext




import open3d
import sys

from EdgebreakerCompression import compress
from EdgebreakerDecompression import decompress
from MeshQualityEvaluation import evaluateWithHausdorff
from Quantization import resizeMesh, writeHeader, printBitString, quantizeVertices, quantizedPositionsToBitstring, normalsToBitstring, clersToBitstring, readVerticesBits
from Encryption import scramble, unscramble, xorifyNormals
from ImportExport import objImporter, objExporter, writeFile, readFile
from tkinter import *


def showMesh(mesh):
	mesh.paint_uniform_color([0.6, 0.6, 0.6])
	open3d.visualization.draw_geometries([mesh])


def preProcess(mesh, doPrint = False):
	if doPrint:
		print(f'Before preprocessing:')
		print(f'Vertices: {len(mesh.vertices)}')
		print(f'Triangles: {len(mesh.triangles)}')

	mesh = mesh.remove_degenerate_triangles()
	mesh = mesh.remove_duplicated_triangles()
	mesh = mesh.remove_duplicated_vertices()
	mesh = mesh.remove_non_manifold_edges()
	mesh = mesh.remove_unreferenced_vertices()

	if doPrint:
		print(f'After preprocessing:')
		print(f'Vertices: {len(mesh.vertices)}')
		print(f'Triangles: {len(mesh.triangles)}')

	return mesh


def cryptoCompress (password, model, filename, outputWidget, outputBar):
    outputWidget.insert(INSERT,'Starting Crypto Compression for\n')
    outputWidget.insert(INSERT,model + '\n')
    outputBar['value'] = 0
    
    k = 10
    outputBar['value'] = 20
    #Load, Quantize and Process our mesh
    outputWidget.insert(INSERT,'Importing...\n')
    originalMesh = objImporter(model)
    bkpMesh = copy.deepcopy(originalMesh)

    outputWidget.insert(INSERT,'Quantizing & Processing...\n')
    quantizeVertices(originalMesh, k)
    preProcess(originalMesh)

    outputWidget.insert(INSERT,'Running EdgeBreaker...\n')
    outputBar['value'] = 40
    #Run EdgeBreaker
    clers, deltas, normals = compress(originalMesh, debug=False)
    bitstring = writeHeader(bkpMesh, k, deltas)

    outputWidget.insert(INSERT,'Writing bitstring...\n')
    outputBar['value'] = 60
    #Add our deltas, normals and clers to our bitsting
    verticesBitstring = quantizedPositionsToBitstring(deltas, k)
    verticesBitstring = verticesBitstring.replace('-', '1') #NOTE: negative numbers can now occur, decompression should take this into account
    #normals = originalMesh.vertex_normals #NOTE: Placeholder normal array
    normalsBitstring = normalsToBitstring(normals, k)
    clersBitstring = clersToBitstring(clers)

    bitstring += verticesBitstring
    bitstring += normalsBitstring
    bitstring += clersBitstring

    print(f'{len(verticesBitstring)} - {len(normalsBitstring)} - {len(clersBitstring)}')

    outputWidget.insert(INSERT,'Encrypting...\n')
    outputBar['value'] = 80
    #From our bitstring, scramble positions and normals
    bitstring = scramble(bitstring, 10, password)
    bitstring = xorifyNormals(bitstring, 10, password)

    printBitString(bitstring)

    print(str(len(deltas)) + " " + str(len(normals)) + " " + str(len(clers)))
    writeFile(bitstring, filename)
    #open3d.visualization.draw_geometries([mesh])
    outputWidget.insert(INSERT,'Done !\n')
    outputWidget.insert(INSERT,'Saved file :\n')
    outputWidget.insert(INSERT,filename + '\n')
    outputBar['value'] = 100

def cryptoExtract (password, filename, modelFilename, outputWidget, outputBar):
    outputWidget.insert(INSERT,'Starting Exctraction for\n')
    outputWidget.insert(INSERT,filename + '\n')

    outputBar['value'] = 0

    bitstring = readFile(filename)

    outputWidget.insert(INSERT,'Decrypting...\n')
    outputBar['value'] = 20
    #Decrypt our bitstring
    bitstring = xorifyNormals(bitstring, 10, password)
    bitstring = unscramble(bitstring, 10, password)


    outputWidget.insert(INSERT,'Reading Data...\n')
    outputBar['value'] = 40
    #Read the bitsting and extract data
    deltas, normals, clers = readVerticesBits(bitstring)

    print(str(len(deltas)) + " " + str(len(normals)) + " " + str(len(clers)))

    outputWidget.insert(INSERT,'Extracting...\n')
    outputBar['value'] = 60
    #Run the Edgebreaker decryption
    decompressedMesh = decompress(clers, deltas, normals, False)

    resizeMesh(bitstring, decompressedMesh, 10)

    outputWidget.insert(INSERT,'Writing file...\n')
    outputBar['value'] = 80
    objExporter(modelFilename, decompressedMesh)

    outputWidget.insert(INSERT,'Done !\n')
    outputWidget.insert(INSERT,'Saved file :\n')
    outputWidget.insert(INSERT,modelFilename + '\n')
    outputBar['value'] = 100

    showMesh(decompressedMesh)

    return decompressedMesh


def main():
    file = ""

    window = tk.Tk()
    window.title("RFCP Crypto Compressor")

    bar = Progressbar(window, length=220)#, style='black.Horizontal.TProgressbar')
    bar['value'] = 0
    bar.grid(column=1, row=15, padx=10, pady=10)

    logBox = scrolledtext.ScrolledText(window)
    logBox.grid(column=1, row=20, pady=15)

    fileLabel = Label(window, text="File")
    fileLabel.grid(column=0, row=10, padx=10, pady=5)

    fileText = Entry(window, width=30)
    fileText.grid(column=1, row=10, padx=10, pady=5)

    def fileBtnClicked ():
        filetypes = (("Compressed RFCP Files","*.rfcp"),("Wavefront .obj file","*.obj"))
        files = filedialog.askopenfilenames(filetypes=filetypes)
        filestr = files[0]
        filestr.replace("{", "")
        filestr.replace("}", "")
        fileText.delete(0, tk.END)
        fileText.insert(0, filestr)
    fileBtn = Button(window, text="Search", command=fileBtnClicked)
    fileBtn.grid(column=2, row=10, padx=10, pady=5)

    passwordLabel = Label(window, text="Password")
    passwordLabel.grid(column=0, row=13, padx=10, pady=5)

    passwordText = Entry(window,width=30, show="•")
    passwordText.grid(column=1, row=13, padx=10, pady=5)

    padding = Label(window, text="")
    padding.grid(column=0, row=14)

    def startBtnClicked ():
        filename = fileText.get()
        password = passwordText.get()
        try:
            if filename.endswith(".rfcp"):
                compressedFilename = filename
                modelFilename = filename[:-5] + "out.obj"
                cryptoExtract(password, compressedFilename, modelFilename, logBox, bar)
            elif filename.endswith(".obj"):
                modelFilename = filename
                compressedFilename = filename[:-4] + ".rfcp"
                cryptoCompress(password, modelFilename, compressedFilename, logBox, bar)
        except Exception as e:
            print(e)
            traceback.print_exc()
            logBox.insert(INSERT,'\nFatal Error, Aborting\n\n')
            bar['value'] = 0

    startBtn = Button(window, text="Start", command=startBtnClicked)
    startBtn.grid(column=0, row=15, padx=10, pady=10)

   

    window.mainloop()

    exit()

    doCompress = True
    doPreProcess = True
    model = "XYZ Dragon.obj"
    filename = "XYZ Dragon.rfcp"    
    #Crypto Compress our mesh
    cryptoCompress("passwd", model, filename)   
    #Crypto Extract our mesh
    cryptoExtract("passwd", filename)   
    exit()


if __name__ == '__main__':
	sys.exit(main())


