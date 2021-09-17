#pragma once

#include "Reconstruction/EnvPointCloud.h"
#include "Reconstruction/PointCloudKNN.h"

/**
*	@file PointCloudClassifier.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 07/03/2021
*/

#define GENERATE_TEMPERATURE_HISTOGRAM true

/**
*	@brief
*/
class PointCloudClassifier
{
protected:
	EnvPointCloud*		_pointCloud;			//!<

	// Ground detection
	PointCloudKNN*		_knn;					//!< 

protected:
	/**
	*	@brief Generates two histograms for vegetation and ground temperature. 
	*/
	void generateTemperatureHistogram(std::vector<unsigned>* pointClass);

public:
	/**
	*	@brief Constructor. 
	*/
	PointCloudClassifier(EnvPointCloud* pointCloud);

	/**
	*	@brief Destructor. 
	*/
	virtual ~PointCloudClassifier();
	
	/**
	*	@brief Tries to classify the point cloud.
	*/
	void load();

	// Getters

	/**
	*	@brief Retrieves normals for point cloud. 
	*/
	void getNormals(std::vector<vec3>& normals);
};

