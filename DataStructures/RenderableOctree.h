#pragma once

/**
*	@file RenderableOctree.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 28/01/2021
*/

#include "Geometry/3D/AABB.h"

#define NUM_OCTREE_NODE_CHILDREN 8

/**
*	@brief 
*/
class RenderableOctree
{
public:
	/**
	*	@brief Returns the bounding boxes for each node of the octree for rendering purposes.
	*	@param aabb Vector of axis-aligned bounding boxes which represent the octree.
	*/
	virtual void getAABBs(std::vector<AABB>& aabb, const unsigned maxLevel = UINT_MAX) = 0;

	/**
	*	@brief Retrieves a point cloud where those nodes with depth > maxLevel are simplified into a point with an averaged color.
	*/
	virtual void retrieveSimplifiedPointCloud(const unsigned maxLevel, std::vector<vec4>& point, std::vector<vec3>& color) = 0;
};

