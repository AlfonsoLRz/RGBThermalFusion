#version 450

#extension GL_ARB_compute_variable_group_size: enable
layout (local_size_variable) in;

#include <Assets/Shaders/Compute/Templates/constraints.glsl>
#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

layout (std430, binding = 0) buffer PointBuffer			{ vec4 points[]; };
layout (std430, binding = 1) buffer SortedIndexBuffer	{ uint sortedIndex[]; };
layout (std430, binding = 2) buffer NeighborBuffer		{ uint neighborIndex[]; };

uniform uint	arraySize;
uniform float	maxWorldDistance;
uniform uint	numNeighbors;
uniform uint	numPoints;
uniform uint	offset;
uniform uint	radius;

float computeDistance(const uint index1, const uint index2)
{
	return distance(points[index1].xyz, points[index2].xyz);
}

bool isSelected(const uint baseIndex, const uint k, const uint currentIndex)
{
	uint currentNeighbor = 0;

	while (currentNeighbor < k)
	{
		if (neighborIndex[baseIndex + currentNeighbor] == currentIndex) return true;
		++currentNeighbor;
	}

	return false;
}

void main()
{
	uint index = gl_GlobalInvocationID.x;
	if (index >= arraySize) return;

	const uint baseIndex = index * numNeighbors;
	index += offset;
	const uint lowerIndex = max(int(index - radius), 0), upperIndex = min(index + radius, numPoints - 1);
	float distance, minDistance = UINT_MAX, maxDistance = -UINT_MAX;
	bool retrievedNeighbor = true;

	for (int k = 0; k < numNeighbors; ++k)
	{
		neighborIndex[baseIndex + k] = UINT_MAX;
	}

	for (int k = 0; k < numNeighbors && retrievedNeighbor; ++k)
	{
		uint currentIndex = lowerIndex;
		minDistance = UINT_MAX;

		while (currentIndex < index)
		{
			distance = computeDistance(index, currentIndex);
			if (distance > maxDistance && distance < minDistance && distance < maxWorldDistance)
			{
				neighborIndex[baseIndex + k] = currentIndex;
				minDistance = distance;
			}

			++currentIndex;
		}

		++currentIndex;

		while (currentIndex <= upperIndex)
		{
			distance = computeDistance(index, currentIndex);
			if (distance > maxDistance && distance < minDistance && distance < maxWorldDistance)
			{
				neighborIndex[baseIndex + k] = currentIndex;
				minDistance = distance;
			}

			++currentIndex;
		}

		maxDistance = minDistance;
		retrievedNeighbor = neighborIndex[baseIndex + k] != UINT_MAX;
	}
}