#include "stdafx.h"
#include "RadiusOctree.h"

/// Public methods

RadiusOctree::RadiusOctree(EnvPointCloud* pointCloud)
	: _root(nullptr), _maxBucketSize(MAX_BUCKET_SIZE)
{
	_points = pointCloud->getPoints();
	_successors = std::vector<uint32_t>(_points->size());
	_aabb = pointCloud->getAABB();

	for (int pointIdx = 0; pointIdx < _points->size(); ++pointIdx)
	{
		_successors[pointIdx] = pointIdx + 1;
	}

	vec3 center = _aabb.min();
	float maxExtent = FLT_MIN;

	for (int idx = 0; idx < 3; ++idx)			// X, Y, Z
	{
		float extent = 0.5f * (_aabb.max()[idx] - _aabb.min()[idx]);
		center[idx] += extent;

		if (extent > maxExtent) maxExtent = extent;
	}

	_root = createOctant(center, maxExtent, 0, _points->size() - 1, _points->size());
}

int32_t RadiusOctree::findNearestNeighbor(const vec3& query, float minDistance)
{
	float maxDistance = FLT_MAX;
	int32_t resultIndex = -1;

	if (!_root) return resultIndex;

	this->findNearestNeighbor(_root, query, minDistance, maxDistance, resultIndex);

	return resultIndex;
}

void RadiusOctree::getAABBs(std::vector<AABB>& aabb, const unsigned maxSize)
{
	this->retrieveNodeData(_root, aabb, maxSize);
}

void RadiusOctree::radiusNeighbors(const vec3& query, float radius, std::vector<uint32_t>& resultIndices)
{
	resultIndices.clear();
	if (!_root) return;

	float powRadius = radius * radius;
	this->radiusNeighbors(_root, query, radius, powRadius, resultIndices);
}

/// Protected methods

RadiusOctree::OctreeNode* RadiusOctree::createOctant(const vec3& center, float extent, uint32_t startIdx, uint32_t endIdx, uint32_t size)
{
	// For a leaf we don't have to change anything; points are already correctly linked or correctly reordered
	OctreeNode* octant = new OctreeNode(this, center, extent, startIdx, endIdx, size);

	// Subdivide subset of points and re-link points according to Morton codes
	if (size > _maxBucketSize)
	{
		const float factor[] = { -0.5f, 0.5f };
		octant->_isLeaf = false;

		std::vector<uint32_t> childStarts(NUM_OCTREE_NODE_CHILDREN, 0);
		std::vector<uint32_t> childEnds(NUM_OCTREE_NODE_CHILDREN, 0);
		std::vector<uint32_t> childSizes(NUM_OCTREE_NODE_CHILDREN, 0);

		// Re-link disjoint child subsets
		uint32_t idx = startIdx;

		for (uint32_t i = 0; i < size; ++i)
		{
			const vec3& point = _points->at(idx);

			// Determine Morton code for each point
			uint32_t mortonCode = 0;
			if (point.x > center.x) mortonCode |= 1;
			if (point.y > center.y) mortonCode |= 2;
			if (point.z > center.z) mortonCode |= 4;

			// Set child starts and update successors...
			if (childSizes[mortonCode] == 0)
			{
				childStarts[mortonCode] = idx;
			}
			else
			{
				_successors[childEnds[mortonCode]] = idx;
			}

			childSizes[mortonCode] += 1;
			childEnds[mortonCode] = idx;
			idx = _successors[idx];
		}

		// Create the child nodes
		float childExtent = 0.5f * extent;
		bool firstTime = true;
		uint32_t lastChildIdx = 0;

		for (uint32_t i = 0; i < NUM_OCTREE_NODE_CHILDREN; ++i)
		{
			if (childSizes[i] == 0) continue;

			float childX = center.x + factor[(i & 1) > 0] * extent;
			float childY = center.y + factor[(i & 2) > 0] * extent;
			float childZ = center.z + factor[(i & 4) > 0] * extent;

			octant->_children[i] = createOctant(vec3(childX, childY, childZ), childExtent, childStarts[i], childEnds[i], childSizes[i]);

			if (firstTime)
			{
				octant->_start = octant->_children[i]->_start;
			}
			else
			{
				_successors[octant->_children[lastChildIdx]->_end] = octant->_children[i]->_start;	// Ensure that also the child ends link to the next child start
			}

			lastChildIdx = i;
			octant->_end = octant->_children[i]->_end;
			firstTime = false;
		}
	}

	return octant;
}

bool RadiusOctree::findNearestNeighbor(OctreeNode* octant, const vec3& query, float minDistance, float& maxDistance, int32_t& resultIndex)
{
	// 1. First descend to leaf and check in leaves
	if (octant->_isLeaf)
	{
		uint32_t idx = octant->_start;
		float sqrMaxDistance = maxDistance * maxDistance;
		float sqrMinDistance = (minDistance < 0) ? minDistance : minDistance * minDistance;

		for (uint32_t i = 0; i < octant->_size; ++i)
		{
			const vec3& point = _points->at(idx);
			vec3 dist3 = point - query;
			float dist = dist3.x * dist3.x + dist3.y * dist3.y + dist3.z * dist3.z;

			if (dist > sqrMinDistance && dist < sqrMaxDistance)
			{
				resultIndex = idx;
				sqrMaxDistance = dist;
			}

			idx = _successors[idx];
		}

		maxDistance = sqrMaxDistance * sqrMaxDistance;

		return octant->inside(query, maxDistance);
	}

	// Determine Morton code for each point...
	uint32_t mortonCode = 0;
	if (query.x > octant->_aabbCenter.x) mortonCode |= 1;
	if (query.y > octant->_aabbCenter.y) mortonCode |= 2;
	if (query.z > octant->_aabbCenter.z) mortonCode |= 4;

	if (octant->_children[mortonCode] != 0)
	{
		if (this->findNearestNeighbor(octant->_children[mortonCode], query, minDistance, maxDistance, resultIndex))
		{
			return true;
		}
	}

	// 2. If current best point completely inside, just return
	float sqrMaxDistance = maxDistance * maxDistance;

	// 3. Check adjacent octants for overlap and check these if necessary
	for (uint32_t c = 0; c < NUM_OCTREE_NODE_CHILDREN; ++c)
	{
		if (c == mortonCode)
		{
			continue;
		}

		if (octant->_children[c] == 0)
		{
			continue;
		}

		if (!octant->_children[c]->overlaps(query, maxDistance, sqrMaxDistance))
		{
			continue;
		}

		if (this->findNearestNeighbor(octant->_children[c], query, minDistance, maxDistance, resultIndex))
		{
			return true;  // Early pruning
		}
	}

	// All children have been checked, check if point is inside the current octant
	return octant->inside(query, maxDistance);
}

void RadiusOctree::radiusNeighbors(OctreeNode* octant, const vec3& query, float radius, float sqrRadius, std::vector<uint32_t>& resultIndices)
{
	// If search ball S(q,r) contains octant, simply add point indexes
	if (octant->contains(query, sqrRadius))
	{
		uint32_t idx = octant->_start;

		for (uint32_t i = 0; i < octant->_size; ++i)
		{
			resultIndices.push_back(idx);
			idx = _successors[idx];
		}

		return;  // Early pruning
	}

	if (octant->_isLeaf)
	{
		uint32_t idx = octant->_start;

		for (uint32_t i = 0; i < octant->_size; ++i)
		{
			const vec3& point = _points->at(idx);
			vec3 dist3 = point - query;
			float dist = dist3.x * dist3.x + dist3.y * dist3.y + dist3.z * dist3.z;

			if (dist < sqrRadius) resultIndices.push_back(idx);
			idx = _successors[idx];
		}

		return;
	}

	// Check whether child nodes are in range
	for (uint32_t children = 0; children < NUM_OCTREE_NODE_CHILDREN; ++children)
	{
		if (!octant->_children[children]) continue;

		if (!octant->_children[children]->overlaps(query, radius, sqrRadius)) continue;
		this->radiusNeighbors(octant->_children[children], query, radius, sqrRadius, resultIndices);
	}
}

void RadiusOctree::retrieveNodeData(OctreeNode* node, std::vector<AABB>& aabb, const unsigned maxSize)
{
	if (node && node->_size > maxSize)
	{
		aabb.push_back(AABB(node->_aabbCenter - vec3(node->_aabbExtent), node->_aabbCenter + vec3(node->_aabbExtent)));

		for (int children = 0; children < NUM_OCTREE_NODE_CHILDREN; ++children)
		{
			if (node->_children[children]) this->retrieveNodeData(node->_children[children], aabb, maxSize);
		}
	}
}


/// OctreeNode

RadiusOctree::OctreeNode::OctreeNode(RadiusOctree* octree, vec3 aabbCenter, float aabbExtent, uint32_t startIdx, uint32_t endIdx, uint32_t size) :
	_octree(octree), _aabbCenter(aabbCenter), _aabbExtent(aabbExtent), _start(startIdx), _end(endIdx), _size(size), _isLeaf(true)
{
	for (int i = 0; i < NUM_OCTREE_NODE_CHILDREN; ++i)
	{
		_children[i] = nullptr;
	}
}

RadiusOctree::OctreeNode::~OctreeNode()
{
	for (OctreeNode* node : _children)
	{
		delete node;
	}
}

bool RadiusOctree::OctreeNode::contains(const vec3& query, float sqRadius)
{
	// We exploit the symmetry to reduce the test to test whether the farthest corner is inside the search ball
	vec3 xyz = glm::abs(query - _aabbCenter);

	// (x, y, z) - (-e, -e, -e) = (x, y, z) + (e, e, e)
	xyz += _aabbExtent;

	return (xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z) < sqRadius;
}

bool RadiusOctree::OctreeNode::inside(const vec3& query, float radius)
{
	// We exploit the symmetry to reduce the test to test whether the farthest corner is inside the search ball
	vec3 xyz = glm::abs(query - _aabbCenter) + radius;

	if (xyz.x > _aabbExtent) return false;
	if (xyz.y > _aabbExtent) return false;
	if (xyz.z > _aabbExtent) return false;

	return true;
}

bool RadiusOctree::OctreeNode::overlaps(const vec3& query, float radius, float sqRadius)
{
	// We exploit the symmetry to reduce the test to testing if its inside the Minkowski sum around the positive quadrant
	vec3 xyz = glm::abs(query - _aabbCenter);
	float maxDist = radius + _aabbExtent;

	// Completely outside, since q' is outside the relevant area
	if (xyz.x > maxDist || xyz.y > maxDist || xyz.z > maxDist) return false;

	int32_t numLessExtent = (xyz.x < _aabbExtent) + (xyz.y < _aabbExtent) + (xyz.x < _aabbExtent);

	// a. inside the surface region of the octant.
	if (numLessExtent > 1) return true;

	// b. checking the corner region && edge region.
	xyz = glm::max(xyz - _aabbExtent, vec3(.0f));

	return (xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z) < sqRadius;
}