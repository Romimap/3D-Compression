from typing import Counter
import huffman
from huffman.huffman import codebook

#reads bitstring considering symbols of n bits
def makeCodebook (bitstring, n):
    dict = {}
    for i in range(0, len(bitstring), n):
        symbol = bitstring[i:i+n]
        if symbol in dict:
            dict[symbol] += 1
        else:
            dict[symbol] = 1

    counterArray = []
    for key in dict:
        counterArray.append((key, dict[key]))

    codebook = huffman.codebook(counterArray)
    print(counterArray)
    print(codebook)
    return codebook