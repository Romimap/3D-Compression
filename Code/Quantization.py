import sys
import os
import open3d
import copy
import numpy
import struct
from codecs import decode


#Thanks google ! https://newbedev.com/how-to-convert-a-binary-string-into-a-float-value 
def float_to_bin(num):
    return bin(struct.unpack('!I', struct.pack('!f', num))[0])[2:].zfill(32)
def bin_to_float(binary):
    return struct.unpack('!f',struct.pack('!I', int(binary, 2)))[0]



def remap (value, minFrom, maxFrom, minTo, maxTo):
    return ((value - minFrom) / (maxFrom - minFrom) ) * (maxTo - minTo) + minTo

#quantize vertices, returning their 92 bits format, and their 2^k bits format
#bits out format : k(4), vertexnb(32), minx(32), miny(32), minz(32), maxx(32), maxy(32), maxz(32), v1(3 * 2^k), v2(3 * 2^k), ... , vn(3 * 2^k)
#                 |                            HEADER (228 bits)                                 |                  VERTICES                  |
def quantizeVertices (vertices, k):
    ansBits = ''
    ansVertices = numpy.asarray(vertices)

    ansBits += '{0:04b}'.format(k)
    ansBits += '{0:032b}'.format(len(ansVertices))

    #Compute AABB
    min = numpy.array([ 999999.9,  999999.9,  999999.9])
    max = numpy.array([-999999.9, -999999.9, -999999.9])

    for vertex in ansVertices:
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
    for vertex in ansVertices:
        vertex[0] = remap(vertex[0], min[0], max[0], 0, 1)
        vertex[1] = remap(vertex[1], min[1], max[1], 0, 1)
        vertex[2] = remap(vertex[2], min[2], max[2], 0, 1)

    #Quantize
    kpow = pow(2, k) - 1
    for vertex in ansVertices:
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
    for vertex in ansVertices:
        vertex[0] = remap(vertex[0], 0, kpow, min[0], max[0])
        vertex[1] = remap(vertex[1], 0, kpow, min[1], max[1])
        vertex[2] = remap(vertex[2], 0, kpow, min[2], max[2])

    return open3d.utility.Vector3dVector(ansVertices), ansBits

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
    

