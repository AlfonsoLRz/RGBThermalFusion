#version 450

#extension GL_ARB_compute_variable_group_size: enable
layout (local_size_variable) in;

#define HORIZONTAL_TERRAIN_ERROR 1.0f / 1000.0f / 10.0f		// Adjust to our scenario scale
#define INT_MAX 0xFFFFFFF									// No previous level on BVH node
#define	MAX_COLLISIONS 10
#define RANDOM_SHINY_POINT_FACTOR 3.0f
#define TEXTURE_OFFSET 1.0f / MAX_COLLISIONS / 2.0f
#define TIME_BOUNDARY_CONST 10.0f
#define VERTICAL_TERRAIN_ERROR .5f / 1000.0f

#define TERRAIN_MASK 1 << 0
#define WATER_MASK 1 << 1
#define VEGETATION_MASK 1 << 2

//const float RETURN_SUCCESS_PROB[MAX_COLLISIONS] = float[MAX_COLLISIONS](0.9f, 0.8f, .7f, .6f, .5f, .4f, .3f, .2f, .1f, .05f);

#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

layout (std430, binding = 0) buffer ClusterBuffer	{ BVHCluster clusterData[]; };
layout (std430, binding = 1) buffer VertexBuffer	{ VertexGPUData vertexData[]; };
layout (std430, binding = 2) buffer FaceBuffer		{ FaceGPUData faceData[]; };
layout (std430, binding = 3) buffer MeshDataBuffer	{ MeshGPUData meshData[]; };
layout (std430, binding = 4) buffer OutputBuffer	{ TriangleCollisionGPUData triangleCollision[]; };
layout (std430, binding = 5) buffer TemporalBuffer	{ TriangleCollisionGPUData collision[]; };
layout (std430, binding = 6) buffer RayBuffer		{ RayGPUData rayData[]; };
layout (std430, binding = 7) buffer CountBuffer		{ uint numCollisions; };

uniform float		distanceWeight;
uniform float		incidenceAngleWeight;
uniform bool		isBathymetric;
uniform float		maximumRange;
uniform uint		maxReturns;
uniform float		moveTerrainHorizontally;
uniform float		moveTerrainVertically;
uniform uint		numClusters;					// Start from last cluster, which belongs to root node
uniform uint		numRays;
uniform float		outlierThreshold;
uniform float		outlierDisplacement;
uniform sampler2D	texReturnSuccessProb;
uniform float		shininessRandWeight;
uniform float		shininessTranslation;

// Random number
float rand(vec2 co){
    return fract(sin(dot(co, vec2(12.9898f, 78.233f))) * 43758.5453f);
}

#include <Assets/Shaders/Compute/Intersections/rayAABB_inters-comp.glsl>
#include <Assets/Shaders/Compute/Intersections/rayTriangle_inters-comp.glsl>

vec3 computeRayDirection(uint index, in vec3 rayDir)
{
	float isWater = clamp(float(meshData[collision[index].modelCompID].surface & WATER_MASK), .0f, 1.0f);
	vec3 refraction = normalize(refract(rayDir, collision[index].intersNormal, 1.33f));

	return rayDir * (1.0f - isWater) + refraction * isWater;
}

// Translates intersection point if a random function selects this point as an outlier
void moveAlongRay(uint index, in vec3 rayDir)
{
	float random = rand(collision[index].intersPoint.xz);						
	float force = random * outlierDisplacement;
	float applyOutlier = sign(clamp(abs(random) - outlierThreshold, 0.0f, 1.0f));		// Values behind threshold are clipped to 0. Outlier or not

	collision[index].intersPoint += -rayDir * force * applyOutlier;
}

void moveShinySurface(uint index, in vec3 rayDir)
{
	float randomModelComp = rand(vec2(collision[index].modelCompID));		
	float randomPoint = rand(collision[index].intersPoint.xy);						

	collision[index].intersPoint += rayDir * meshData[collision[index].modelCompID].shininess * (shininessTranslation + shininessRandWeight * randomModelComp + randomPoint / RANDOM_SHINY_POINT_FACTOR);
}

void moveTerrain(uint index, in vec3 rayStartingPoint, in vec3 rayDir)
{
	// Horizontal error
	vec3 axis = normalize(vec3(rand(collision[index].intersPoint.yx), .0f, rand(collision[index].intersPoint.xz)));
	float strength = abs(rayStartingPoint.y - collision[index].intersPoint.y) * rand(collision[index].intersNormal.xy);
	float isTerrain = float(meshData[collision[index].modelCompID].surface & TERRAIN_MASK);

	collision[index].intersPoint += isTerrain * moveTerrainHorizontally * axis * strength * HORIZONTAL_TERRAIN_ERROR * collision[index].distance;

	strength = asin(clamp(collision[index].intersNormal.y / length(collision[index].intersNormal), -1.0f, 1.0f));

	collision[index].intersPoint += isTerrain * moveTerrainVertically * vec3(.0f, 1.0f, .0f) * strength * VERTICAL_TERRAIN_ERROR;
}

// Checks collision between a ray and BVH built from scene geometry & topology
bool checkCollision(uint index, inout RayGPUData ray, uint returnNumber, inout uint finalIndex, inout bool isCollisionValid)
{
	const float discardProb = rand(ray.orig.xz);

	if (discardProb > texture(texReturnSuccessProb, vec2((float(returnNumber) / MAX_COLLISIONS) + TEXTURE_OFFSET, .5f)).r)
	{
		return false;
	}

	// Our own triangle collision; once collided then its saved
	collision[index].faceIndex = -1;
	collision[index].distance = INT_MAX;

	// Initialize stack
	bool collided = false;
	int currentIndex = 0;
	uint toExplore[100];
	toExplore[currentIndex] = numClusters - 1;

	while (currentIndex >= 0)
	{
		BVHCluster cluster = clusterData[toExplore[currentIndex]];

		if (rayAABBIntersection(ray, cluster.minPoint, cluster.maxPoint, collision[index].distance))
		{
			if (cluster.faceIndex != INT_MAX)
			{
				if (rayTriangleIntersection(index, faceData[cluster.faceIndex], ray))
				{
					collision[index].faceIndex = cluster.faceIndex;								// Collision point is already set
					collision[index].modelCompID = faceData[cluster.faceIndex].modelCompID;
					collision[index].returnNumber = returnNumber;
					collision[index].intensity = clamp(meshData[faceData[cluster.faceIndex].modelCompID].reflectance.x -
														abs(collision[index].angle) * incidenceAngleWeight -
														collision[index].distance * distanceWeight,
														.0f, 1.0f);

					collided = true;
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

	vec3 rayDir = ray.dir;
	isCollisionValid = collided && (!isBathymetric || (meshData[collision[index].modelCompID].surface & WATER_MASK) == 0);

	if (collided)
	{
		ray.orig = collision[index].intersPoint + ray.dir * EPSILON;
		ray.dir = computeRayDirection(index, ray.dir);
		ray.dest = ray.orig + ray.dir;
	}

	if (isCollisionValid)
	{
		moveShinySurface(index, rayDir);							// Shiny surface translation
		moveTerrain(index, ray.startingPoint.xyz, rayDir);
		moveAlongRay(index, rayDir);								// Simulate outlier points

		finalIndex = atomicAdd(numCollisions, 1);
		triangleCollision[finalIndex] = collision[index];
	}

	return collided;
}
		

void main()
{
	const uint index = gl_GlobalInvocationID.x;
	if (index >= numRays)
	{
		return;
	}

	uint returnNumber = 0, finalIndex = index;
	uint collisionIndices[MAX_COLLISIONS];
	bool continueRay = true, isCollisionValid;
	RayGPUData ray = rayData[index];					// First ray is given by outside
	ray.startingPoint.w = EPSILON;

	while (continueRay)
	{
		continueRay = checkCollision(index, ray, returnNumber, finalIndex, isCollisionValid);	

		collisionIndices[returnNumber] = finalIndex;
		returnNumber += uint(isCollisionValid);

		continueRay = continueRay && (((meshData[collision[index].modelCompID].surface & VEGETATION_MASK) != 0) || (isBathymetric && (meshData[collision[index].modelCompID].surface & WATER_MASK) != 0))
					  && returnNumber < maxReturns;
	}

	int numCollisions = int(returnNumber) - 1;
	while (numCollisions >= 0)
	{
		triangleCollision[collisionIndices[numCollisions]].numReturns = returnNumber;
		--numCollisions;
	}
}