#include "stdafx.h"
#include "PointCloudClassifier.h"

#include "DataStructures/PointCloudBVH.h"
#include "Graphics/Application/PointCloudParameters.h"
#include "Utilities/Histogram.h"

// [Public methods]

PointCloudClassifier::PointCloudClassifier(EnvPointCloud* pointCloud) : _pointCloud(pointCloud), _knn(nullptr)
{
}

PointCloudClassifier::~PointCloudClassifier()
{
	delete _knn;
}

void PointCloudClassifier::load()
{
	std::vector<vec4> normals;
	std::vector<uint32_t> groundIndices;
	std::vector<unsigned> pointClass (_pointCloud->getNumberOfPoints());

	std::fill(pointClass.begin(), pointClass.end(), PointCloudParameters::VEGETATION);
	
	_knn = new PointCloudKNN(_pointCloud->getPoints(), _pointCloud->getAABB());
	_knn->computeNormals(normals);
	_pointCloud->setNormals(normals);

	// Distinguish soil from rest of scene
	PointCloudBVH* bvh = new PointCloudBVH(_pointCloud->getPoints(), &normals, _pointCloud->getAABB(), 0.08f, .05f, vec3(.0f, .0f, 1.0f), _pointCloud->getAABB().size().z * 1.5f);
	bvh->buildBVH();
	bvh->testVisibilityForGroundDetection(groundIndices);

	for (uint32_t& groundIndex: groundIndices)
	{
		pointClass[groundIndex] = PointCloudParameters::GROUND;
	}

	_pointCloud->setPointClass(pointClass);
	this->generateTemperatureHistogram(&pointClass);
}

/// [Protected methods]

void PointCloudClassifier::generateTemperatureHistogram(std::vector<unsigned>* pointClass)
{
	std::vector<vec4>* temperature = _pointCloud->getColors(EnvPointCloud::TEMPERATURE);
	vec2 temperatureRange = _pointCloud->getTemperatureRange();

	// Histogram data
	std::vector<float> groundTemp, vegetationTemp;

	for (int pointIndex = 0; pointIndex < temperature->size(); ++pointIndex)
	{
		if (temperature->at(pointIndex).w > glm::epsilon<float>())
		{
			if (pointClass->at(pointIndex) == PointCloudParameters::VEGETATION)
			{
				vegetationTemp.push_back(temperature->at(pointIndex).y);
			}
			else if (pointClass->at(pointIndex) == PointCloudParameters::GROUND)
			{
				groundTemp.push_back(temperature->at(pointIndex).y);
			}
		}
	}

	Histogram* groundHistogram = new Histogram(&groundTemp);
	groundHistogram->buildHistogram(unsigned(temperatureRange.y - temperatureRange.x) * 5, temperatureRange.x, temperatureRange.y, true);
	groundHistogram->exportLatex("Histogram/Ground.txt");

	Histogram* vegetationHistogram = new Histogram(&vegetationTemp);
	vegetationHistogram->buildHistogram(unsigned(temperatureRange.y - temperatureRange.x) * 5, temperatureRange.x, temperatureRange.y, true);
	vegetationHistogram->exportLatex("Histogram/Vegetation.txt"); 
}
