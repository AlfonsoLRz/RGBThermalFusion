#include "stdafx.h"
#include "OcclusionAugmentation.h"

#include "DataStructures/MultiInstancePointCloudBVH.h"
#include "Utilities/ChronoUtilities.h"

// [Public methods]

ThermalAugmentation::VolatileAugmentationData* OcclusionAugmentation::augmentatePointCloud(EnvPointCloud* pointCloud, RadiusOctree* pointCloudOctree)
{
	float temperature;
	vec2 point2D; vec3 point3D, color;
	VolatileAugmentationData* colorData = this->getColorData(pointCloud);
	std::vector<EnvCamera*>* rgbCameras = EnvCamera::getCameras(EnvImage::RGB_THERMAL_IMAGE);
	std::vector<vec4>* points = pointCloud->getPoints();
	AABB sceneAABB = pointCloud->getAABB();
	MultiInstancePointCloudBVH* bvh = new MultiInstancePointCloudBVH(7000000);

#if COMPUTE_ERROR
	std::unordered_map<GLuint, std::vector<EnvCamera*>> pointCamera;
#endif	
	
	for (EnvCamera* rgbCamera : *rgbCameras)
	{
		if (rgbCamera->isRegistered(EnvImage::THERMAL_IMAGE))
		{
			std::vector<uint32_t> candidatePointsIndices, visiblePoints;
			std::vector<vec4> candidatePoints;
			
			// 1) Retrieve points which could be visible
			const vec3 cameraPosition = rgbCamera->getLocalPosition();
			pointCloudOctree->radiusNeighbors(vec3(cameraPosition.x, cameraPosition.y, sceneAABB.center().z), SEARCH_RADIUS, candidatePointsIndices);

			// 2) Gather candidate points
			unsigned indexInc = 0;
			candidatePoints.resize(candidatePointsIndices.size());
			for (uint32_t& index : candidatePointsIndices)
			{
				candidatePoints[indexInc++] = points->at(index);
			}

			// 3) Check if those points are visible on GPU
			bvh->reserveSpace(&candidatePoints, rgbCamera, sceneAABB);
			bvh->buildBVH();
			bvh->testVisibility(visiblePoints);

			// 4) Retrieve color for visible points
			for (uint32_t index : visiblePoints)
			{
				point3D = candidatePoints.at(index);
				point2D = rgbCamera->transformTo2D(vec4(point3D, 1.0f));

				if (rgbCamera->getImage()->retrieveColor(EnvImage::THERMAL_IMAGE, point2D, color))
				{
					colorData->_color[EnvPointCloud::THERMAL][candidatePointsIndices[index]] += vec4(color, .0f);
					++colorData->_numContributions[EnvPointCloud::THERMAL][candidatePointsIndices[index]];

#if COMPUTE_ERROR
					this->insertPointCameraHashMap(pointCamera, candidatePointsIndices[index], rgbCamera);
#endif

					if (rgbCamera->getImage()->getTemperature(point2D, temperature))
					{
						colorData->_color[EnvPointCloud::TEMPERATURE][candidatePointsIndices[index]] += vec4(vec3(temperature), 1.0f);
						++colorData->_numContributions[EnvPointCloud::TEMPERATURE][candidatePointsIndices[index]];
					}
				}
			}
		}
	}

	delete bvh;

	// Calculate average color
	unsigned numColors, meanContributions = 0;
	const unsigned numPoints = pointCloud->getNumberOfPoints();
	float maxTemperature = FLT_MIN, minTemperature = FLT_MAX, meanTemperature = .0f;

	for (int pointIdx = 0; pointIdx < numPoints; ++pointIdx)
	{
		numColors = glm::clamp(colorData->_numContributions[EnvPointCloud::THERMAL][pointIdx], unsigned(1), UINT_MAX);
		colorData->_color[EnvPointCloud::THERMAL][pointIdx] = colorData->_color[EnvPointCloud::THERMAL][pointIdx] / float(numColors) / 255.0f;
		colorData->_color[EnvPointCloud::THERMAL][pointIdx].w = colorData->_numContributions[EnvPointCloud::THERMAL][pointIdx] > 0;
		
		numColors = glm::clamp(colorData->_numContributions[EnvPointCloud::TEMPERATURE][pointIdx], unsigned(1), UINT_MAX);
		colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx] = colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx] / float(numColors);
		colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx].w = colorData->_numContributions[EnvPointCloud::TEMPERATURE][pointIdx] > 0;

		if (colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx].w > glm::epsilon<float>())
		{
			maxTemperature = glm::max(maxTemperature, colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx].x);
			minTemperature = glm::min(minTemperature, colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx].x);
		}
	}

	for (int temperature = 0; temperature < colorData->_color[EnvPointCloud::TEMPERATURE].size(); ++temperature)
	{
		colorData->_color[EnvPointCloud::TEMPERATURE][temperature] = vec4((colorData->_color[EnvPointCloud::TEMPERATURE][temperature].x - minTemperature) / (maxTemperature - minTemperature), colorData->_color[EnvPointCloud::TEMPERATURE][temperature].x, .0f,
																			 colorData->_color[EnvPointCloud::TEMPERATURE][temperature].w);

		if (colorData->_color[EnvPointCloud::TEMPERATURE][temperature].w > glm::epsilon<float>())
		{
			meanTemperature += colorData->_color[EnvPointCloud::TEMPERATURE][temperature].x;
			++meanContributions;
		}
	}

	pointCloud->setTemperatureRange(minTemperature, maxTemperature);
	pointCloud->setMean(meanContributions > 0 ? meanTemperature / meanContributions : .0f);

#if COMPUTE_ERROR
	this->computeErrors(colorData, points, pointCamera);

#if COMPUTE_HISTOGRAM
	this->computeHistogram(colorData, points, pointCamera);
#endif
#endif

	return colorData;
}