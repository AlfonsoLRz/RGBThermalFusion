#pragma once

#include "Reconstruction/ThermalAugmentation.h"

/**
*	@file NaiveThermalAugmentation.h
*	@authors Alfonso L�pez Ruiz (alr00048@red.ujaen.es)
*	@date 26/03/2021
*/

/**
*	@brief Defines a na�ve algorithm where thermal information is appended without any additional comprobation about occlusion.
*/
class NaiveThermalAugmentation : public ThermalAugmentation
{
public:
	/**
	*	@brief Adds thermal information to the given point cloud.
	*/
	virtual VolatileAugmentationData* augmentatePointCloud(EnvPointCloud* pointCloud, RadiusOctree* pointCloudOctree);
};

