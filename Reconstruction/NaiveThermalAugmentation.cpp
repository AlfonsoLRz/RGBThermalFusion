#include "stdafx.h"
#include "NaiveThermalAugmentation.h"

// [Public methods]

ThermalAugmentation::VolatileAugmentationData* NaiveThermalAugmentation::augmentatePointCloud(EnvPointCloud* pointCloud, RadiusOctree* pointCloudOctree)
{
	ThermalAugmentation::VolatileAugmentationData* colorData = this->getColorData(pointCloud);
	AABB aabb = pointCloud->getAABB();
	std::vector<vec4>* points = pointCloud ? pointCloud->getPoints() : nullptr;
	std::vector<EnvCamera*>* cameras = EnvCamera::getCameras(EnvImage::RGB_THERMAL_IMAGE);

	if (pointCloud)
	{
#if COMPUTE_ERROR
		std::unordered_map<GLuint, std::vector<EnvCamera*>> pointCamera;
#endif		
		float height = pointCloud->getAABB().center().z, maxTemperature = FLT_MIN, minTemperature = FLT_MAX, tempColor, meanTemperature = .0f;
		vec3 thermalColor;
		unsigned numColorsThermal, numColorsTemperature, unregisteredPoints = 0, meanContributions = 0;
		EnvCamera* thermalCamera;

		for (int cameraIdx = 0; cameraIdx < cameras->size(); ++cameraIdx)
		{
			EnvCamera* camera = cameras->at(cameraIdx);

			if (camera->isRegistered(EnvImage::THERMAL_IMAGE))
			{
				const vec3 cameraLocalPosition = camera->getLocalPosition();
				std::vector<uint32_t> indices;
				bool isPointVisible = false;

				pointCloudOctree->radiusNeighbors(vec3(cameraLocalPosition.x, cameraLocalPosition.y, height), SEARCH_RADIUS, indices);

				for (uint32_t index : indices)
				{
					const vec2 imagePoint = camera->transformTo2D(points->at(index));
					if (camera->getImage()->retrieveColor(EnvImage::THERMAL_IMAGE, imagePoint, thermalColor))
					{
						colorData->_color[EnvPointCloud::THERMAL][index] += vec4(thermalColor, .0f);
						++colorData->_numContributions[EnvPointCloud::THERMAL][index];

#if COMPUTE_ERROR
						this->insertPointCameraHashMap(pointCamera, index, camera);
#endif

						if (camera->getImage()->getTemperature(imagePoint, tempColor))
						{
							colorData->_color[EnvPointCloud::TEMPERATURE][index].x += tempColor;
							++colorData->_numContributions[EnvPointCloud::TEMPERATURE][index];

							maxTemperature = glm::max(maxTemperature, tempColor);
							minTemperature = glm::min(minTemperature, tempColor);
						}
					}
				}
			}
		}

		for (int pointIdx = 0; pointIdx < points->size(); ++pointIdx)
		{
			numColorsThermal = glm::clamp(colorData->_numContributions[EnvPointCloud::THERMAL][pointIdx], unsigned(1), UINT_MAX);					// Avoid division by zero
			numColorsTemperature = glm::clamp(colorData->_numContributions[EnvPointCloud::TEMPERATURE][pointIdx], unsigned(1), UINT_MAX);		
			
			colorData->_color[EnvPointCloud::THERMAL][pointIdx] = vec4(colorData->_color[EnvPointCloud::THERMAL][pointIdx].x / 255.0f / numColorsThermal, colorData->_color[EnvPointCloud::THERMAL][pointIdx].y / 256.0f / numColorsThermal,
																		 colorData->_color[EnvPointCloud::THERMAL][pointIdx].z / 255.0f / numColorsThermal, colorData->_numContributions[EnvPointCloud::THERMAL][pointIdx] > 0 ? 1.0f : .0f);
			colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx] = vec4(vec3(colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx].x / numColorsTemperature), colorData->_numContributions[EnvPointCloud::TEMPERATURE][pointIdx] > 0 ? 1.0f : .0f);
			colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx] = vec4((colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx].x - minTemperature) / (maxTemperature - minTemperature), colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx].x, .0f,
																			  colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx].w);

			if (colorData->_numContributions[EnvPointCloud::THERMAL][pointIdx] == 0) ++unregisteredPoints;

			if (colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx].w > glm::epsilon<float>())
			{
				meanTemperature += colorData->_color[EnvPointCloud::TEMPERATURE][pointIdx].x;
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

		std::cout << "Number of Unregistered Points: " << unregisteredPoints << " from " << points->size() << "; " << float(unregisteredPoints) / points->size() * 100.0f << "%" << std::endl;
	}

	return colorData;
}
