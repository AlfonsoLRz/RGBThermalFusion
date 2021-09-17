#pragma once

#include "Geometry/3D/AABB.h"

#define USE_SVD				true
#define ESTIMATE_NORMAL_GPU	false

/**
*	@file PointCloudKNN.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 07/03/2021
*/

/**
*	@brief
*/
class PointCloudKNN
{
protected:
	const static unsigned K_NEIGHBORS;				//!<
	const static unsigned MAX_POINT_CLOUD_SIZE;		//!< 
	const static unsigned SEARCH_RADIUS;			//!<

protected:
	struct VolatileGPUData
	{
		GLuint		_arraySize;						//!< Size of point cloud buffer	
		GLuint		_pointCloudSSBO;				//!< SSBO of geometry
		GLuint		_mortonCodesSSBO;				//!< Morton codes for each point
		GLuint		_neighborsSSBO;					//!<
		GLuint		_normalSSBO;					//!< 
		GLuint		_sortedIndicesSSBO;				//!< Sorted buffer of points

		/**
		*	@brief
		*/
		VolatileGPUData() :
			_pointCloudSSBO(-1), _mortonCodesSSBO(-1), _neighborsSSBO(-1), _normalSSBO(-1), _sortedIndicesSSBO(-1) {}

		/**
		*	@brief Destructor.
		*/
		~VolatileGPUData()
		{
			const int numBuffers = 5;

			GLuint toDeleteBuffers[numBuffers] = { _pointCloudSSBO, _mortonCodesSSBO, _neighborsSSBO, _normalSSBO, _sortedIndicesSSBO };
			glDeleteBuffers(numBuffers, toDeleteBuffers);
		}
	};

protected:
	AABB				_aabb;					//!<
	std::vector<vec4>*	_points;				//!<

	// [Algorithm settings]
	float				_maxDistance;			//!< 
	unsigned			_numNeighbors;			//!<
	unsigned			_searchRadius;			//!<
	vec3				_viewPoint;				//!< 

	// [GPU Data]
	VolatileGPUData*	_gpuData;				//!<

protected:
	/**
	*	@brief Builds the buffers which are necessary to seach K nearest neighbours.
	*/
	void aggregateSSBOData();

	/**
	*	@brief
	*/
	GLuint calculateMortonCodes();

	/**
	*	@brief Computes normals for each point in GPU.
	*/
	void computeNormalsCrossEstimation(std::vector<vec4>& normals, const unsigned offset);

	/**
	*	@brief Computes normals for each point through CGAL library.
	*/
	void computeNormalsCGAL(std::vector<vec4>& normals);
	
	/**
	*	@brief  
	*/
	void computeNormalsGPU(std::vector<vec4>& normals);

	/**
	*	@brief Computes normals for each point through PCL library.
	*/
	void computeNormalsPCL(std::vector<vec4>& normals);

	/**
	*	@brief Computes normals through SVD plane fitting.
	*/
	void computeNormalsSVD(std::vector<vec4>& normals);

	/**
	*	@brief  
	*/
	void searchKNeighbors(const unsigned offset);

	/**
	*	@brief Rearranges the triangles to sort them by their morton codes.
	*/
	GLuint sortFacesByMortonCode();

public:
	/**
	*	@brief Constructor. 
	*/
	PointCloudKNN(std::vector<vec4>* points, const AABB& aabb, const unsigned kNeighbors = K_NEIGHBORS, const unsigned searchRadius = SEARCH_RADIUS);

	/**
	*	@brief Destructor.
	*/
	virtual ~PointCloudKNN();

	/**
	*	@brief Calculates normals for point cloud through KNN search. 
	*/
	void computeNormals(std::vector<vec4>& normals);
};

