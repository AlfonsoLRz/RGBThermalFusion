#version 450

#extension GL_ARB_compute_variable_group_size: enable
layout (local_size_variable) in;

#include <Assets/Shaders/Compute/Templates/constraints.glsl>
#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

layout (std430, binding = 0) buffer ClusterBuffer	{ BVHCluster	clusterData[]; };
layout (std430, binding = 1) buffer VertexBuffer	{ vec4			point[]; };
layout (std430, binding = 2) buffer OutputBuffer	{ uint			collidedPoint[]; };
layout (std430, binding = 3) buffer CountBuffer		{ uint			numCollisions; };

uniform vec3		cameraPosition;
uniform uint		numClusters;					
uniform uint		numPoints;

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

bool raySphereCollision(in RayGPUData ray, vec3 minPoint, vec3 maxPoint, out float distance)
{
	const vec3 center	= (maxPoint + minPoint) / 2.0f;
	const float radius	= center.x - minPoint.x;

	vec3 oc = ray.origin - center;
    float b = dot(oc, ray.direction);
    float c = dot(oc, oc) - radius * radius;
    float h = b * b - c;

    if (h < -.001f) return false;

    h = sqrt(h);
	distance = min(-b - h, -b + h);

    return true;
}

// Checks collision between a ray and BVH built from scene geometry & topology
uint checkCollision(uint index, in RayGPUData ray)
{
	// Initialize stack
	uint toExplore[100];
	int currentIndex			= 0;
	float minDistance			= float(UINT_MAX), currentDistance;
	uint collisionIndex			= UINT_MAX;
	toExplore[currentIndex]		= numClusters - 1;

	while (currentIndex >= 0)
	{
		BVHCluster cluster = clusterData[toExplore[currentIndex]];

		if (rayAABBCollision(ray, cluster.minPoint, cluster.maxPoint))
		{
			if (cluster.faceIndex != UINT_MAX)
			{
				if (raySphereCollision(ray, cluster.minPoint, cluster.maxPoint, currentDistance) && (currentDistance < minDistance))
				{
					collisionIndex = cluster.faceIndex;
					minDistance = currentDistance;
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

	RayGPUData ray;
	ray.origin		= cameraPosition;
	ray.direction	= normalize(point[index].xyz - cameraPosition);
	ray.destination = point[index].xyz;

	const uint collisionIndex = checkCollision(index, ray);	

	if (collisionIndex == index)
	{
		const uint finalIndex		= atomicAdd(numCollisions, 1);
		collidedPoint[finalIndex]	= index;
	}
}