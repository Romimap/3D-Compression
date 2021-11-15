import sys
import os
import open3d
import copy
import numpy
import struct

def binary(num):
    return ''.join('{:0>8b}'.format(c) for c in struct.pack('!f', num))

def remap (value, minFrom, maxFrom, minTo, maxTo):
    return ((value - minFrom) / (maxFrom - minFrom) ) * (maxTo - minTo) + minTo

#quantize vertices, returning their 92 bits format, and their 2^k bits format
#bits out format : vertexnb(32), minx(32), miny(32), minz(32), maxx(32), maxy(32), maxz(32), v1(3 * 2^k), v2(3 * 2^k), ... , vn(3 * 2^k)
#                 |                            HEADER (224 bits)                           |                  VERTICES                  |
def quantizeVertices (vertices, k):
    ansBits = ''
    ansVertices = numpy.asarray(vertices)

    ansBits += '{0:032b}'.format(ansVertices.size)

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

    ansBits += binary(numpy.float32(min[0]))
    ansBits += binary(numpy.float32(min[1]))
    ansBits += binary(numpy.float32(min[2]))
    ansBits += binary(numpy.float32(max[0]))
    ansBits += binary(numpy.float32(max[1]))
    ansBits += binary(numpy.float32(max[2]))

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

    print(ansVertices)

    return open3d.utility.Vector3dVector(ansVertices), ansBits

    
