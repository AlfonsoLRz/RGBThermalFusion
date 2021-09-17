#version 450

#extension GL_ARB_compute_variable_group_size: enable
layout (local_size_variable) in;

#include <Assets/Shaders/Compute/Templates/constraints.glsl>
#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

layout (std430, binding = 0) buffer ClusterBuffer	{ BVHCluster	clusterData[]; };
layout (std430, binding = 1) buffer VertexBuffer	{ vec4			point[]; };
layout (std430, binding = 2) buffer NormalBuffer	{ vec4			normal[]; };
layout (std430, binding = 3) buffer OutputBuffer	{ uint			collidedPoint[]; };
layout (std430, binding = 4) buffer CountBuffer		{ uint			numCollisions; };

uniform vec3		cameraOrientation;
uniform vec3		groundOrientation;
uniform float		normalEpsilon;
uniform uint		numClusters;					
uniform uint		numPoints;
uniform float		searchRadius;

// Slabs method
bool rayAABBCollision(in RayGPUData ray, vec3 minPoint, vec3 maxPoint)
{
	vec3 tMin	= (minPoint - ray.origin) / ray.direction;
    vec3 tMax	= (maxPoint - ray.origin) / ray.direction;
    vec3 t1		= min(tMin, tMax);
    vec3 t2		= max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar	= min(min(t2.x, t2.y), t2.z);

    return tFar >= tNear;
}

bool raySphereCollision(in RayGPUData ray, vec3 minPoint, vec3 maxPoint, uint threadIndex, uint pointIndex, out float pointDistance)
{
	const vec3 center	= (maxPoint + minPoint) / 2.0f;
	const float radius	= center.x - minPoint.x;

	vec3 oc = ray.origin - center;
    float b = dot(oc, ray.direction);
    float c = dot(oc, oc) - radius * radius;
    float h = b * b - c;

    if (h < -.001f) return false;

	h = sqrt(h);
	pointDistance = min(-b - h, -b + h);

    return true;
}

// Checks collision between a ray and BVH built from scene geometry & topology
uint checkCollision(uint index, in RayGPUData ray, out float maxDistance)
{
	// Initialize stack
	uint toExplore[100];
	int currentIndex			= 0;
	float currentDistance		= 0;
	uint collisionIndex			= UINT_MAX;
	toExplore[currentIndex]		= numClusters - 1;
	maxDistance					= float(-UINT_MAX);

	while (currentIndex >= 0)
	{
		BVHCluster cluster = clusterData[toExplore[currentIndex]];

		if (rayAABBCollision(ray, cluster.minPoint, cluster.maxPoint))
		{
			if (cluster.faceIndex != UINT_MAX)
			{
				if (raySphereCollision(ray, cluster.minPoint, cluster.maxPoint, index, cluster.faceIndex, currentDistance) && (currentDistance > maxDistance))
				{
					collisionIndex = cluster.faceIndex;
					maxDistance = currentDistance;
				}
			}
			else
			{
				toExplore[currentIndex] = cluster.prevIndex1;
				toExplore[++currentIndex] = cluster.prevIndex2;
				++currentIndex;										// Prevent --currentIndex instead of branching
			}
		}

		--currentIndex;
	}

	return collisionIndex;
}
		

void main()
{
	const uint index = gl_GlobalInvocationID.x;
	if (index >= numPoints)
	{
		return;
	}

	// Check if normal is near to UP vector
	float dotProduct = dot(groundOrientation, normal[index].xyz);

	if (dotProduct > 1.0f - normalEpsilon)
	{
		RayGPUData ray;
		ray.origin = point[index].xyz + cameraOrientation;
		ray.direction = normalize(-cameraOrientation);
		ray.destination = point[index].xyz;

		float maxDistance, pointDistance = float(UINT_MAX);
		const uint collisionIndex = checkCollision(index, ray, maxDistance);

		if (collisionIndex != UINT_MAX)
		{
			pointDistance = distance(point[index].xyz, point[collisionIndex].xyz);
		}

		if (collisionIndex == UINT_MAX || pointDistance < searchRadius * 2.0f)
		{
			const uint finalIndex = atomicAdd(numCollisions, 1);
			collidedPoint[finalIndex] = index;
		}
	}
}