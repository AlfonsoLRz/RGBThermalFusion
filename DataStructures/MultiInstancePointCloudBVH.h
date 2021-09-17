#pragma once

#include "Graphics/Core/DrawBVH.h"
#include "Reconstruction/EnvPointCloud.h"
#include "Reconstruction/EnvCamera.h"

/**
*	@file MultiInstancePointCloudBVH.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 15/04/2021
*/

/**
*	@brief Builds a BVH whose points are spheres.
*/
class MultiInstancePointCloudBVH
{
protected:
	const static unsigned	BVH_BUILDING_RADIUS;		//!<

protected:
	// [Point cloud]
	AABB					_aabb;						//!<
	std::vector<vec4>*		_points;					//!<

	// [Camera data - GSD]
	EnvCamera*			_camera;					//!<
	float					_focalLength;				//!<
	float					_height;					//!<
	float					_imageWidth;				//!<
	float					_sensorWidth;				//!<

	// [GPU buffers]
	GLuint					_arraySizeCount;
	GLuint					_cinBuffer;
	GLuint					_clusterSSBO;				//!< BVH nodes
	GLuint					_groupGeometrySSBO;			//!< SSBO of geometry
	GLuint					_inCurrentPosition;
	GLuint*					_indices;
	GLuint					_indicesBufferID_1;
	GLuint					_mergedCluster;				//!< A merged cluster is always valid, but the opposite situation is not fitting
	GLuint					_mortonCodeBuffer;
	GLuint					_nBitsBufferID;
	GLuint					_neighborIndex;				//!< Nearest neighbor search
	GLuint					_numNodesCount;				//!< Number of currently added nodes, which increases as the clusters are merged
	GLuint					_outCurrentPosition;
	GLuint					_pBitsBufferID;
	GLuint					_positionBufferID;
	GLuint					_prefixScan;				//!< Final position of each valid cluster for the next loop iteration
	GLuint					_tempClusterSSBO;			//!< Temporary buffer for BVH construction
	GLuint					_validCluster;				//!< Clusters which takes part of next loop iteration
	GLuint					_triangleCollisionSSBO;

protected:
	/**
	*	@brief Builds the buffers which are necessary to build the BVH.
	*/
	void aggregateSSBOData();

	/**
	*	@brief
	*/
	void buildClusterBuffer(const GLuint sortedFaces);

	/**
	*	@brief
	*/
	GLuint calculateMortonCodes();

	/**
	*	@brief
	*/
	void generateBVH();

	/**
	*	@brief Rearranges the triangles to sort them by their morton codes.
	*/
	GLuint sortFacesByMortonCode(const GLuint mortonCodes);

public:
	/**
	*	@brief
	*/
	MultiInstancePointCloudBVH(const unsigned arraySize);

	/**
	*	@brief
	*/
	~MultiInstancePointCloudBVH();

	/**
	*	@brief
	*/
	void buildBVH();

	/**
	*	@brief
	*/
	void reserveSpace(std::vector<vec4>* points, EnvCamera* camera, const AABB& aabb);

	/**
	*	@brief Test visibility of every point which was included in the BVH.
	*/
	void testVisibility(std::vector<uint32_t>& indices);
};

