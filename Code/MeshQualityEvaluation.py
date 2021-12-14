
import math
import open3d
import sys


def calculateDistance(p1, p2):
	return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2) + math.pow(p1[2] - p2[2], 2))


def evaluateWithHausdorff(originalMesh, compressedMesh):
	hausdorffDistance = 0

	totalIterations = len(originalMesh.vertices) * len(compressedMesh.vertices)
	print("Total iterations: {totalIterations}")
    
	i = 0
	progress = 1
	for p1 in compressedMesh.vertices:
		minDist = float('inf')
		for p2 in originalMesh.vertices:
			minDist = min(calculateDistance(p1, p2), minDist)
			i += 1
			if ((100 * i) / totalIterations) > progress:
				print(f'Progress: {progress}% ({i}/{totalIterations})')
				progress += 1
		hausdorffDistance = max(minDist, hausdorffDistance)
    
	return hausdorffDistance


def main():
	...


if __name__ == '__main__':
	sys.exit(main())
