import Code.bcolors
import sys
import os
from numpy.core.numeric import indices
import open3d
import copy
import numpy
import struct
import math
import random
from codecs import decode

headerSize = 228


# https://newbedev.com/how-to-convert-a-binary-string-into-a-float-value
def float_to_bin(num):
    return bin(struct.unpack('!I', struct.pack('!f', num))[0])[2:].zfill(32)


# https://newbedev.com/how-to-convert-a-binary-string-into-a-float-value
def bin_to_float(binary):
    return struct.unpack('!f', struct.pack('!I', int(binary, 2)))[0]


# https://stackoverflow.com/a/26127012
# Edited so it uses numpy instead of vanilla arrays
def fibonacci_sphere(samples=131072): #NOTE: might be 131071
    points = numpy.zeros([samples, 3])
    phi = math.pi * (3. - math.sqrt(5.))  # golden angle in radians

    for i in range(samples):
        y = 1 - (i / float(samples - 1)) * 2  # y goes from 1 to -1
        radius = math.sqrt(1 - y * y)  # radius at y

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius
        z = math.sin(theta) * radius

        points[i] = numpy.array([x, y, z])

    return points


def closestNormalID(kdFibSphere, normal):
    [k, idx, _] = kdFibSphere.search_knn_vector_3d(normal, 5)
    return idx[0]

def remap(value, minFrom, maxFrom, minTo, maxTo):
    return ((value - minFrom) / (maxFrom - minFrom)) * (maxTo - minTo) + minTo


#quantize vertices, returning their bitstream header
#bits out format : k(4), vertexnb(32), minx(32), miny(32), minz(32), maxx(32), maxy(32), maxz(32), v1(3 * 2^k), v2(3 * 2^k), ... , vn(3 * 2^k)
#                 |                            HEADER (228 bits)                                 |                  VERTICES                  |
def quantizeVertices(mesh, k):
    vertices = numpy.asarray(mesh.vertices)

    print("vnb @ quantization: " + str(len(vertices)))

    bitstring = ''
    bitstring += '{0:04b}'.format(k)
    bitstring += '{0:032b}'.format(len(vertices))
    print("bin : " + bitstring[4:])

    # * * * * * * * * * *
    # * * POSITIONS * * *
    # * * * * * * * * * *
    #Compute AABB
    min = numpy.array([999999.9, 999999.9, 999999.9])
    max = numpy.array([-999999.9, -999999.9, -999999.9])

    for vertex in vertices:
        if vertex[0] > max[0]: max[0] = vertex[0]
        if vertex[1] > max[1]: max[1] = vertex[1]
        if vertex[2] > max[2]: max[2] = vertex[2]
        if vertex[0] < min[0]: min[0] = vertex[0]
        if vertex[1] < min[1]: min[1] = vertex[1]
        if vertex[2] < min[2]: min[2] = vertex[2]

    bitstring += float_to_bin(numpy.float64(min[0]))
    bitstring += float_to_bin(numpy.float64(min[1]))
    bitstring += float_to_bin(numpy.float64(min[2]))
    bitstring += float_to_bin(numpy.float64(max[0]))
    bitstring += float_to_bin(numpy.float64(max[1]))
    bitstring += float_to_bin(numpy.float64(max[2]))

    #Normalize coordinates into a unit AABB
    for vertex in vertices:
        vertex[0] = remap(vertex[0], min[0], max[0], 0, 1)
        vertex[1] = remap(vertex[1], min[1], max[1], 0, 1)
        vertex[2] = remap(vertex[2], min[2], max[2], 0, 1)

    #Quantize
    kpow = pow(2, k) - 1
    for vertex in vertices:
        x = int(round(kpow * vertex[0]))
        y = int(round(kpow * vertex[1]))
        z = int(round(kpow * vertex[2]))
        vertex[0] = x
        vertex[1] = y
        vertex[2] = z

    return bitstring

#Returns the bitstring representing the positions of our mesh
def quantizedPositionsToBitstring(vertices, k):
    bitstring = ''
    for vertex in vertices:
        bitstring += str('{0:0' + str(k) + 'b}').format(int(vertex[0]))
        bitstring += str('{0:0' + str(k) + 'b}').format(int(vertex[1]))
        bitstring += str('{0:0' + str(k) + 'b}').format(int(vertex[2]))
    return bitstring

#Returns the bitstring representing the normals of our mesh
def normalsToBitstring(normals, k):
    bitstring = ''
    # * * * * * * * * * *
    # * * * NORMALS * * *
    # * * * * * * * * * *
    fibSphere = fibonacci_sphere()
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(fibSphere)    
    kdFibSphere = open3d.geometry.KDTreeFlann(pcd)

    for normal in normals:
        id = closestNormalID(kdFibSphere, normal)
        bitstring += '{0:017b}'.format(int(id))
    return bitstring


def printbin(bitstring, start, end, colorstart = 0, colorend = 0, rangevalue = 8):
    r = 0
    for i in range(start, end, 32):
        print(str(i).ljust(8) + ": ", end='')
        for j in range(0, 32, 8):
            for k in range(0, 8):
                if (i+j+k >= start and i+j+k < end):
                    if (i+j+k >= colorstart and i+j+k < colorend):
                        print(f"{Code.bcolors.bcolors.OKGREEN}" + bitstring[i+j+k], end='')
                    else:
                        if (int(r / rangevalue) % 2 == 0):
                            print(f"{Code.bcolors.bcolors.OKCYAN}" + bitstring[i+j+k], end='')
                        else:
                            print(f"{Code.bcolors.bcolors.OKBLUE}" + bitstring[i+j+k], end='')
                    r += 1
            print(' ', end='')
        print(f"{Code.bcolors.bcolors.ENDC}")
    print(str(end - 1).ljust(8) + ": END")

def printBitString(bitstring):
    # K
    k = int(bitstring[0:4], 2)
    # Vertex Count
    vertexCount = int(bitstring[4:36], 2)

    print("K: " + str(k))
    print("VertexCount: " + str(vertexCount))

    printbin(bitstring, 0, 4)
    printbin(bitstring, 4, 228)

    n = headerSize
    printbin(bitstring, n, n + (12*k), n, n + 3 * k, k)
    print ('...')
    n += 3 * k * vertexCount
    printbin(bitstring, n - (12 * k), n, n - 3 * k, n, k)

    kn = 17
    printbin(bitstring, n, n + (kn * 10), n, n + kn, 17)
    print ('...')
    n += 17 * vertexCount
    printbin(bitstring, n - (kn * 10), n, n - kn, n, 17)






def readVerticesBits(bitstring):
    print("bitstring len: " + str(len(bitstring)))

    # K
    k = int(bitstring[0:4], 2)
    # Vertex Count
    vertexCount = int(bitstring[4:36], 2)

    # AABB
    minbitstring = bitstring[36:132]
    maxbitstring = bitstring[132:228]

    minx = bin_to_float(minbitstring[0:32])
    miny = bin_to_float(minbitstring[32:64])
    minz = bin_to_float(minbitstring[64:96])

    maxx = bin_to_float(maxbitstring[0:32])
    maxy = bin_to_float(maxbitstring[32:64])
    maxz = bin_to_float(maxbitstring[64:96])

    min = numpy.array([minx, miny, minz])
    max = numpy.array([maxx, maxy, maxz])

   

    # Vertices
    n = headerSize
    kpow = pow(2, k) - 1
    vertices = numpy.zeros([vertexCount, 3])

    for i in range(0, vertexCount):
        x = int(bitstring[n:n + k], 2)
        n += k
        y = int(bitstring[n:n + k], 2)
        n += k
        z = int(bitstring[n:n + k], 2)
        n += k
        vertex = numpy.array([
            remap(x, 0, kpow, min[0], max[0]),
            remap(y, 0, kpow, min[1], max[1]),
            remap(z, 0, kpow, min[2], max[2])
        ])
        vertices[i] = vertex

    # Normals
    fibSphere = fibonacci_sphere()
    kn = 17
    normals = numpy.zeros([vertexCount, 3])
    for i in range(0, vertexCount):
        x = int(bitstring[n:n + kn], 2)
        n += kn
        normals[i] = fibSphere[x]


    
    return vertices, normals

