#pragma once

#include "DataStructures/PointCloudBVH.h"
#include "DataStructures/RadiusOctree.h"
#include "Reconstruction/EnvCamera.h"
#include "Reconstruction/EnvPointCloud.h"

#define COMPUTE_ERROR true
#define COMPUTE_HISTOGRAM true
#define SAVE_BRDF_POINT true

/**
*	@file ThermalAugmentation.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 26/03/2021
*/

/**
*	@brief Defines a base class for any algorithm which joins RGB point clouds and thermal information.
*/
class ThermalAugmentation
{
protected:
	const static float		DISTANCE_DIVISOR;									//!<
	const static float		IMAGE_SCALE[NUM_IMAGE_TYPES];						//!<
	const static unsigned	SEARCH_RADIUS;										//!<

public:
	/**
	 *	@brief
	 */
	struct VolatileAugmentationData
	{
		std::vector<vec4>	_color[EnvPointCloud::NUM_COLORS];				//!<
		std::vector<GLuint>	_numContributions[EnvPointCloud::NUM_COLORS];		//!<

		/**
		*	@brief
		*/
		void calloc(const EnvPointCloud::PointColorTypes colorType, const unsigned size);
	};

protected:
	/**
	*	@return Pointer to an initialized object of color data for a point cloud.
	*/
	VolatileAugmentationData* getColorData(EnvPointCloud* pointCloud);

	// Error calculus
	
	/**
	*	@brief Computes several error measurements with respect to image color. 
	*/
	void computeErrors(ThermalAugmentation::VolatileAugmentationData* colorData, std::vector<vec4>* points, std::unordered_map<GLuint, std::vector<EnvCamera*>>& pointCamera);

	/**
	*	@brief Computes several error measurements with respect to image color.
	*/
	void computeHistogram(ThermalAugmentation::VolatileAugmentationData* colorData, std::vector<vec4>* points, std::unordered_map<GLuint, std::vector<EnvCamera*>>& pointCamera);

	/**
	*	@brief  
	*/
	void insertPointCameraHashMap(std::unordered_map<GLuint, std::vector<EnvCamera*>>& pointCamera, GLuint index, EnvCamera* camera);

	/**
	*	@brief  
	*/
	void saveBRDF(const vec4& point, const std::vector<EnvCamera*>* cameras);

public:
	/**
	*	@brief Adds thermal information to the given point cloud. 
	*/
	virtual VolatileAugmentationData* augmentatePointCloud(EnvPointCloud* pointCloud, RadiusOctree* pointCloudOctree) = 0;
};

