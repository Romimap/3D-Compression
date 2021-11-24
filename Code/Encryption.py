import random

from Code.Quantization import headerSize

SALTPOS = "salty"
SALTNRM = "salty"


def xorifyNormals (bitstring, k, key):
    random.seed(key + SALTNRM)
    vertexNb = int(bitstring[4:36], 2)
    bitstring = list(bitstring)
    startindex = headerSize + (3 * k * vertexNb)
    for k in range(startindex, len(bitstring)):
        if int(bitstring[k]) == random.randint(0, 1):
            bitstring[k] = '0'
        else:
            bitstring[k] = '1'

    return "".join(bitstring)




def gettransposition(key, vertexNb):
    random.seed(key + SALTPOS)
    transpositionsX = []
    transpositionsY = []
    transpositionsZ = []

    for _ in range(vertexNb):
        transpositionsX.append(
            (random.randint(0, vertexNb - 1), random.randint(0, vertexNb - 1)))
        transpositionsY.append(
            (random.randint(0, vertexNb - 1), random.randint(0, vertexNb - 1)))
        transpositionsZ.append(
            (random.randint(0, vertexNb - 1), random.randint(0, vertexNb - 1)))

    return transpositionsX, transpositionsY, transpositionsZ

def scramble(bitstring, k, key):
    vertexNb = int(bitstring[4:36], 2)
    print ("vnb: " + str(vertexNb))
    print ("bin: " + bitstring[4:36])
    transpositionsX, transpositionsY, transpositionsZ = gettransposition(key, vertexNb)

    xindex = list(range(0, vertexNb))
    yindex = list(range(0, vertexNb))
    zindex = list(range(0, vertexNb))

    print ('tx: ' + str(len(transpositionsX)) + '[' + str(min(transpositionsX)) + ', ' + str(max(transpositionsX)) + ']')
    print ('ty: ' + str(len(transpositionsY)) + '[' + str(min(transpositionsY)) + ', ' + str(max(transpositionsY)) + ']')
    print ('tz: ' + str(len(transpositionsZ)) + '[' + str(min(transpositionsZ)) + ', ' + str(max(transpositionsZ)) + ']')


    for (a, b) in transpositionsX:
        xindex[a], xindex[b] = xindex[b], xindex[a]

    for (a, b) in transpositionsY:
        yindex[a], yindex[b] = yindex[b], yindex[a]

    for (a, b) in transpositionsZ:
        zindex[a], zindex[b] = zindex[b], zindex[a]

    vertexstring = ''
    for i in range(vertexNb):
        xpos = headerSize + (xindex[i] * 3 * k)
        ypos = headerSize + (yindex[i] * 3 * k) + (k)
        zpos = headerSize + (zindex[i] * 3 * k) + (2 * k)
        vertexstring += bitstring[xpos:xpos+k]
        vertexstring += bitstring[ypos:ypos+k]
        vertexstring += bitstring[zpos:zpos+k]

    vertexlen = len(vertexstring)
    bitstring = bitstring[:headerSize] + vertexstring + bitstring[headerSize+vertexlen:]

    return bitstring


def unscramble(bitstring, k, key):
    vertexNb = int(bitstring[4:36], 2)
    print ("vnb: " + str(vertexNb))
    print ("bin: " + bitstring[4:36])
    transpositionsX, transpositionsY, transpositionsZ = gettransposition(key, vertexNb)

    xindex = list(range(0, vertexNb))
    yindex = list(range(0, vertexNb))
    zindex = list(range(0, vertexNb))

    print ('tx: ' + str(len(transpositionsX)) + '[' + str(min(transpositionsX)) + ', ' + str(max(transpositionsX)) + ']')
    print ('ty: ' + str(len(transpositionsY)) + '[' + str(min(transpositionsY)) + ', ' + str(max(transpositionsY)) + ']')
    print ('tz: ' + str(len(transpositionsZ)) + '[' + str(min(transpositionsZ)) + ', ' + str(max(transpositionsZ)) + ']')


    for (a, b) in reversed(transpositionsX):
        xindex[a], xindex[b] = xindex[b], xindex[a]

    for (a, b) in reversed(transpositionsY):
        yindex[a], yindex[b] = yindex[b], yindex[a]

    for (a, b) in reversed(transpositionsZ):
        zindex[a], zindex[b] = zindex[b], zindex[a]

    vertexstring = ''
    for i in range(vertexNb):
        xpos = headerSize + (xindex[i] * 3 * k)
        ypos = headerSize + (yindex[i] * 3 * k) + (k)
        zpos = headerSize + (zindex[i] * 3 * k) + (2 * k)
        vertexstring += bitstring[xpos:xpos+k]
        vertexstring += bitstring[ypos:ypos+k]
        vertexstring += bitstring[zpos:zpos+k]

    vertexlen = len(vertexstring)
    bitstring = bitstring[:headerSize] + vertexstring + bitstring[headerSize+vertexlen:]

    return bitstring