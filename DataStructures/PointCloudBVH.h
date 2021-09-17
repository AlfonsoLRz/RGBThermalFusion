#pragma once

#include "Graphics/Core/DrawBVH.h"
#include "Reconstruction/EnvPointCloud.h"
#include "Reconstruction/EnvCamera.h"

/**
*	@file PointCloudBVH.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 23/02/2021
*/

/**
*	@brief Builds a BVH whose points are spheres.
*/
class PointCloudBVH
{
protected:
	struct BVHVolatileGPUData
	{
		GLuint		_groupGeometrySSBO;										//!< SSBO of geometry
		GLuint		_clusterSSBO;											//!< BVH nodes
		GLuint		_tempClusterSSBO;										//!< Temporary buffer for BVH construction
		GLuint		_mortonCodesSSBO;										//!< Morton codes for each triangle

		/**
		*	@brief
		*/
		BVHVolatileGPUData() :
			_groupGeometrySSBO(-1), _clusterSSBO(-1), _tempClusterSSBO(-1), _mortonCodesSSBO(-1) {}

		/**
		*	@brief Destructor.
		*/
		~BVHVolatileGPUData()
		{
			const int numBuffers = 4;

			GLuint toDeleteBuffers[numBuffers] = { _groupGeometrySSBO, _clusterSSBO, _tempClusterSSBO, _mortonCodesSSBO };
			glDeleteBuffers(numBuffers, toDeleteBuffers);
		}
	};

protected:
	const static unsigned	BVH_BUILDING_RADIUS;	//!<

protected:
	// [Point cloud]
	AABB					_aabb;					//!<
	std::vector<vec4>* _points;				//!<
	std::vector<vec4>* _normals;				//!<

	// [GPU Data]
	BVHVolatileGPUData* _gpuData;				//!<

	// [Camera data - GSD]
	EnvCamera* _camera;				//!<
	float					_focalLength;			//!<
	float					_height;				//!<
	float					_imageWidth;			//!<
	float					_sensorWidth;			//!<

	// [Camera data - Search with fixed radius]
	vec3					_cameraDirection;		//!<
	float					_normalEpsilon;			//!<
	float					_searchRadius;			//!< 

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
	void buildClusterBuffer4CustomRadius(const GLuint sortedFaces);

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
	PointCloudBVH(std::vector<vec4>* points, EnvCamera* camera, const AABB& aabb);

	/**
	*	@brief
	*/
	PointCloudBVH(std::vector<vec4>* points, std::vector<vec4>* normals, const AABB& aabb, const float searchRadius, const float normalEps, const vec3& cameraDirection, const float cameraHeight);

	/**
	*	@brief
	*/
	~PointCloudBVH();

	/**
	*	@brief
	*/
	void buildBVH();

	/**
	*	@brief Test visibility of every point which was included in the BVH.
	*/
	void testVisibility(std::vector<uint32_t>& indices);

	/**
	*	@brief Test visibility of every point which was included in the BVH.
	*/
	void testVisibilityForGroundDetection(std::vector<uint32_t>& indices);
};