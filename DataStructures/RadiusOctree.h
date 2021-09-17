#pragma once

#include "DataStructures/RenderableOctree.h"
#include "Geometry/3D/AABB.h"
#include "Reconstruction/EnvCamera.h"
#include "Reconstruction/EnvPointCloud.h"

#define MAX_BUCKET_SIZE 10

/**
*	@file RadialOctree.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 07/01/2021
*/

/**
*	@brief
*/
class RadiusOctree
{
protected:
	class OctreeNode;

	OctreeNode*					_root;					//!<

	// [Octree content]
	std::vector<vec4>*			_points;				//!<

	// [Octree metadata]
	AABB						_aabb;					//!<
	uint8_t						_maxBucketSize;			//!< Maximum capacity of a node
	std::vector<uint32_t>		_successors;			//!<

protected:
	/**
	*	@brief
	*/
	OctreeNode* createOctant(const vec3& center, float extent, uint32_t startIdx, uint32_t endIdx, uint32_t size);

	/**
	*	@brief
	*/
	bool findNearestNeighbor(OctreeNode* octant, const vec3& query, float minDistance, float& maxDistance, int32_t& resultIndex);

	/**
	*	@brief
	*/
	void radiusNeighbors(OctreeNode* octant, const vec3& query, float radius, float sqrRadius, std::vector<uint32_t>& resultIndices);

	/**
	*	@brief ---
	*	@param node Node where we're searching.
	*	@param aabb Retrieved aligned axis bounding boxes.
	*/
	void retrieveNodeData(OctreeNode* node, std::vector<AABB>& aabb, const unsigned maxSize);

public:
	/**
	*	@brief
	*/
	RadiusOctree(EnvPointCloud* pointCloud);

	/**
	*	@brief
	*/
	int32_t findNearestNeighbor(const vec3& query, float minDistance = -1);

	/**
	*	@return
	*/
	AABB getAABB() { return _aabb; }

	/**
	*	@brief Returns the bounding boxes for each node of the octree for rendering purposes.
	*	@param aabb Vector of axis-aligned bounding boxes which represent the octree.
	*/
	virtual void getAABBs(std::vector<AABB>& aabb, const unsigned maxSize = INT_MAX);

	/**
	*	@brief
	*/
	void radiusNeighbors(const vec3& query, float radius, std::vector<uint32_t>& resultIndices);

	// Setters

	/**
	*	@brief
	*/
	void setMaximumBucketSize(const unsigned maxBucketSize) { _maxBucketSize = maxBucketSize; }

protected:
	class OctreeNode
	{
	public:
		// Metadata
		RadiusOctree*		_octree;
		uint32_t			_start, _end, _size;

		// AABB
		vec3				_aabbCenter;
		float				_aabbExtent;

		// Children
		OctreeNode*			_children[NUM_OCTREE_NODE_CHILDREN];
		bool				_isLeaf;

	public:
		/**
		*	@brief
		*/
		OctreeNode(RadiusOctree* octree, vec3 aabbCenter, float aabbExtent, uint32_t startIdx, uint32_t endIdx, uint32_t size);

		/**
		*	@brief Destructor.
		*/
		~OctreeNode();

		/**
		*	@brief
		*	@return
		*/
		bool contains(const vec3& query, float sqRadius);

		/**
		*	@brief
		*	@return
		*/
		bool inside(const vec3& query, float radius);

		/**
		*	@brief
		*	@return
		*/
		bool overlaps(const vec3& query, float radius, float sqRadius);
	};
};


