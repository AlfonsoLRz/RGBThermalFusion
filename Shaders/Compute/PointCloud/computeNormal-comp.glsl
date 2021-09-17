#version 450

#extension GL_ARB_compute_variable_group_size: enable
layout (local_size_variable) in;

#include <Assets/Shaders/Compute/Templates/constraints.glsl>
#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

layout (std430, binding = 0) buffer PointBuffer			{ vec4 points[]; };
layout (std430, binding = 1) buffer NeighborBuffer		{ uint neighborIndex[]; };
layout (std430, binding = 2) buffer NormalBuffer		{ vec4 normal[]; };

uniform uint arraySize;
uniform uint numNeighbors;
uniform uint offset;
uniform vec3 viewpoint;

void main()
{
	const uint index = gl_GlobalInvocationID.x;
	if (index >= arraySize) return;

	const uint baseIndex = index * numNeighbors;

	uint numCrossProducts = 0;
	vec3 v1, v2;
	vec4 normalTemp = vec4(.0f);

	for (int ki = 0; ki < numNeighbors; ++ki)
	{
		for (int kj = ki + 1; kj < numNeighbors; ++kj)
		{	
			if (neighborIndex[baseIndex + ki] != UINT_MAX && neighborIndex[baseIndex + kj] != UINT_MAX)
			{
				v1 = normalize(points[neighborIndex[baseIndex + ki]].xyz - points[offset + index].xyz);
				v2 = normalize(points[neighborIndex[baseIndex + kj]].xyz - points[offset + index].xyz);

				normalTemp = normalTemp + vec4(normalize(cross(v1, v2)), .0f);
				++numCrossProducts;
			}
		}
	}

	if (numCrossProducts > 0)
	{
		normal[index] = vec4(normalize(normalTemp.xyz / numCrossProducts), .0f);

		const vec3 viewpointVector = normalize(viewpoint);
		const float angle = acos(dot(normal[index].xyz, viewpointVector));

		if (angle >= PI / 2.0f)
		{
			normal[index] = -normal[index];
		}
	}
}