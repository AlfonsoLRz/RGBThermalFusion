#pragma once

#include "Reconstruction/ThermalAugmentation.h"

/**
*	@file OcclusionAugmentation.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 26/03/2021
*/

/**
*	@brief Defines an algorithm based on GPU where points are presented through volumes and can then be occluded by others.
*/
class OcclusionAugmentation : public ThermalAugmentation
{
	/**
	*	@brief Adds thermal information to the given point cloud.
	*/
	virtual VolatileAugmentationData* augmentatePointCloud(EnvPointCloud* pointCloud, RadiusOctree* pointCloudOctree);
};

