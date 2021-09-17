#pragma once

#include "Reconstruction/ImageDepthBuffer.h"
#include "Reconstruction/ThermalAugmentation.h"

/**
*	@file DepthBufferAugmentation.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 26/03/2021
*/

/**
*	@brief Defines a visibility test based on depth buffer structures for each camera.
*/
class DepthBufferAugmentation : public ThermalAugmentation
{
protected:
	/**
	*	@brief
	*/
	void checkDepthBufferCPU(EnvPointCloud* pointCloud, EnvCamera* camera, std::vector<uint32_t>& candidatePoints, std::vector<vec2>& points2D, ImageDepthBuffer* depthBuffer, VolatileAugmentationData* colorData, std::unordered_map<GLuint, std::vector<EnvCamera*>>* pointCamera);

public:
	/**
	*	@brief Adds thermal information to the given point cloud.
	*/
	virtual VolatileAugmentationData* augmentatePointCloud(EnvPointCloud* pointCloud, RadiusOctree* pointCloudOctree);
};

