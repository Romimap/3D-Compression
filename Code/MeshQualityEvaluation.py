
import math
import open3d
import sys


def closestPointID(kdtree, point):
	[k, idx, dist] = kdtree.search_knn_vector_3d(point, 1)
	return idx[0]


def calculateDistance(p1, p2):
	return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2) + math.pow(p1[2] - p2[2], 2))


def evaluateWithHausdorff(originalMesh, compressedMesh):
	hausdorffDistance = 0

	# Create a KdTree to quickly find closest points
	pcd = open3d.geometry.PointCloud(originalMesh.vertices)
	kdtree = open3d.geometry.KDTreeFlann(pcd)

	# Calculate Hausdorff distance from compressedMesh to originalMesh
	for p in compressedMesh.vertices:
		distance = calculateDistance(p, originalMesh.vertices[closestPointID(kdtree, p)])
		hausdorffDistance = max(distance, hausdorffDistance)

	return hausdorffDistance


def main():
	...


if __name__ == '__main__':
	sys.exit(main())
