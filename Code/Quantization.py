import sys
import os
from numpy.core.numeric import indices
import open3d
import copy
import numpy
import struct
import math
from codecs import decode

# https://newbedev.com/how-to-convert-a-binary-string-into-a-float-value 
def float_to_bin(num):
    return bin(struct.unpack('!I', struct.pack('!f', num))[0])[2:].zfill(32)

# https://newbedev.com/how-to-convert-a-binary-string-into-a-float-value 
def bin_to_float(binary):
    return struct.unpack('!f',struct.pack('!I', int(binary, 2)))[0]

# https://stackoverflow.com/a/26127012
# Edited so it uses numpy instead of vanilla arrays
def fibonacci_sphere(samples=100):
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

def closestNormalID (fibSphere, normal):
    closestid = -1
    closest = 999999
    #NOTE : Could use a kdTree here
    #NOTE : We should use a kdTree its flocking slow
    for i in range(len(fibSphere)):
        fibNormal = fibSphere[i]
        dx, dy, dz = (fibNormal[0] - normal[0]), (fibNormal[1] - normal[1]), (fibNormal[2] - normal[2])
        d = dx*dx + dy*dy + dz*dz   
        if d < closest:
            closest = d
            closestid = i
    return closestid




def remap (value, minFrom, maxFrom, minTo, maxTo):
    return ((value - minFrom) / (maxFrom - minFrom) ) * (maxTo - minTo) + minTo

#quantize vertices, returning their 92 bits format, and their 2^k bits format
#bits out format : k(4), vertexnb(32), minx(32), miny(32), minz(32), maxx(32), maxy(32), maxz(32), v1(3 * 2^k), v2(3 * 2^k), ... , vn(3 * 2^k)
#                 |                            HEADER (228 bits)                                 |                  VERTICES                  |
def quantizeVertices (mesh, k):
    vertices = numpy.asarray(mesh.vertices)
    normals = numpy.asarray(mesh.vertex_normals)

    ansBits = ''

    ansBits += '{0:04b}'.format(k)
    ansBits += '{0:032b}'.format(len(vertices))

    # * * * * * * * * * * 
    # * * POSITIONS * * * 
    # * * * * * * * * * * 
    #Compute AABB
    min = numpy.array([ 999999.9,  999999.9,  999999.9])
    max = numpy.array([-999999.9, -999999.9, -999999.9])

    for vertex in vertices:
        if vertex[0] > max[0]: max[0] = vertex[0]
        if vertex[1] > max[1]: max[1] = vertex[1]
        if vertex[2] > max[2]: max[2] = vertex[2]
        if vertex[0] < min[0]: min[0] = vertex[0]
        if vertex[1] < min[1]: min[1] = vertex[1]
        if vertex[2] < min[2]: min[2] = vertex[2]

    ansBits += float_to_bin(numpy.float64(min[0]))
    ansBits += float_to_bin(numpy.float64(min[1]))
    ansBits += float_to_bin(numpy.float64(min[2]))
    ansBits += float_to_bin(numpy.float64(max[0]))
    ansBits += float_to_bin(numpy.float64(max[1]))
    ansBits += float_to_bin(numpy.float64(max[2]))

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
        ansBits += str('{0:0' + str(k) + 'b}').format(x)
        ansBits += str('{0:0' + str(k) + 'b}').format(y)
        ansBits += str('{0:0' + str(k) + 'b}').format(z)

    #Remap the data to the original AABB
    for vertex in vertices:
        vertex[0] = remap(vertex[0], 0, kpow, min[0], max[0])
        vertex[1] = remap(vertex[1], 0, kpow, min[1], max[1])
        vertex[2] = remap(vertex[2], 0, kpow, min[2], max[2])


    # * * * * * * * * * * 
    # * * * NORMALS * * * 
    # * * * * * * * * * * 
    fibSphere = fibonacci_sphere()
    for normal in normals:
        id = closestNormalID(fibSphere, normal)
        ansBits += '{0:017b}'.format(int(id))
        normal = fibSphere[id]


    print ("before")
    print (normals)
    return ansBits

def readVerticesBits(bitstring):

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
    n = 228
    kpow = pow(2, k) - 1
    vertices = numpy.zeros([vertexCount, 3])
    for i in range(vertexCount):
        x = int(bitstring[n:n+k], 2)
        n += k
        y = int(bitstring[n:n+k], 2)
        n += k
        z = int(bitstring[n:n+k], 2)
        n += k
        vertex = numpy.array([remap(x, 0, kpow, min[0], max[0]), remap(y, 0, kpow, min[1], max[1]), remap(z, 0, kpow, min[2], max[2])])
        vertices[i] = vertex

    # Normals
    fibSphere = fibonacci_sphere()
    kn = 17
    normals = numpy.zeros([vertexCount, 3])
    for i in range(vertexCount):

        x = int(bitstring[n:n+kn], 2)
        n += kn

        normals[i] = fibSphere[x]

    print ("after")
    print (normals)

    return vertices, normals

