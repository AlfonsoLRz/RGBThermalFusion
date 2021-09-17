#version 450

#extension GL_ARB_compute_variable_group_size: enable
layout (local_size_variable) in;

#define INF 2^32

layout (std430, binding = 0) buffer InputBuffer { uvec2 depthBuffer[]; };

uniform uint bufferSize;
uniform uint workGroupSize;

void main()
{
	uint threadID = gl_LocalInvocationID.x + gl_WorkGroupID.x * workGroupSize;

	while (threadID < bufferSize)
	{
		depthBuffer[threadID] = ivec2(1, 10);

		threadID += gl_NumWorkGroups.x * workGroupSize;
	}
}