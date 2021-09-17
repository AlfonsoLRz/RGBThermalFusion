struct VertexGPUData
{
	vec3 position;
	vec3 normal;
	vec2 textCoord;
	vec3 tangent;
};

struct FaceGPUData
{
	uvec3	vertices;
	uint	modelCompID;

	vec3	minPoint;
	vec3	maxPoint;
	vec3    normal;
};

struct MeshGPUData
{
	vec2	padding1;
	uint	numVertices;
	uint	startIndex;
};

struct BVHCluster
{
	vec3 minPoint;
	uint prevIndex1;

	vec3 maxPoint;
	uint prevIndex2;

	uint faceIndex;
};

// Intersections section

struct RayGPUData 
{
	vec3	origin;
	vec3	destination;
	vec3	direction;
};

struct TriangleCollisionGPUData
{
	vec3	point;
	uint	faceIndex;

	vec3	normal;
	float	distance;

	vec2	textCoord;
	uint	modelCompID;	
	float	padding1;

	vec3	tangent;
	float	padding2;
};