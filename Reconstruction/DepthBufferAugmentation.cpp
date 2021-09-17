#include "stdafx.h"
#include "DepthBufferAugmentation.h"

#include "Utilities/ChronoUtilities.h"

// [Public methods]

ThermalAugmentation::VolatileAugmentationData* DepthBufferAugmentation::augmentatePointCloud(EnvPointCloud* pointCloud, RadiusOctree* pointCloudOctree)
{
	VolatileAugmentationData* colorData = this->getColorData(pointCloud);
	std::vector<EnvCamera*>* rgbCameras = EnvCamera::getCameras(EnvImage::RGB_THERMAL_IMAGE);
	ImageDepthBuffer* depthBuffer = new ImageDepthBuffer(EnvCamera::getOriginalImageSize(EnvImage::RGB_THERMAL_IMAGE) * IMAGE_SCALE[EnvImage::RGB_THERMAL_IMAGE], 1);
	AABB sceneAABB = pointCloud->getAABB();
	std::vector<vec4>* points = pointCloud ? pointCloud->getPoints() : nullptr;
	std::unordered_map<GLuint, std::vector<EnvCamera*>> pointCamera;
	std::vector<vec2> points2D(pointCloud->getNumberOfPoints());

	for (EnvCamera* rgbCamera : *rgbCameras)
	{
		if (rgbCamera->isRegistered(EnvImage::THERMAL_IMAGE))
		{
			std::vector<uint32_t> candidatePoints;

			// 1) Retrieve points which could be visible
			const vec3 cameraPosition = rgbCamera->getLocalPosition();
			pointCloudOctree->radiusNeighbors(vec3(cameraPosition.x, cameraPosition.y, sceneAABB.center().z), SEARCH_RADIUS, candidatePoints);

			// 2) Check in GPU which of them can be rendered
			this->checkDepthBufferCPU(pointCloud, rgbCamera, candidatePoints, points2D, depthBuffer, colorData, &pointCamera);
		}
	}

	// Calculate normalized colors
	unsigned numColors, meanContributions = 0;
	const unsigned numPoints = pointCloud->getNumberOfPoints();
	float maxTemperature = FLT_MIN, minTemperature = FLT_MAX, meanTemperature = .0f;

	for (int pointIdx = 0; pointIdx < numPoints; ++pointIdx)
	{
		if (!colorData->_numContributions[EnvPointCloud::THERMAL][pointIdx]) continue;

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

	delete depthBuffer;

	return colorData;
}

// [Protected methods]

void DepthBufferAugmentation::checkDepthBufferCPU(EnvPointCloud* pointCloud, EnvCamera* camera, std::vector<uint32_t>& candidatePoints, std::vector<vec2>& points2D, ImageDepthBuffer* depthBuffer, 
												  ThermalAugmentation::VolatileAugmentationData* colorData, std::unordered_map<GLuint, std::vector<EnvCamera*>>* pointCamera)
{
	float temperature;
	vec2 point2D; vec3 point3D, color;
	std::vector<vec4>* points = pointCloud->getPoints();
	
	depthBuffer->resetDepth();
	depthBuffer->setCameraPosition(camera->getLocalPosition());
	
	for (unsigned& pointIndex: candidatePoints)
	{
		point3D = points->at(pointIndex);
		point2D = camera->transformTo2D(vec4(point3D, 1.0f));
		depthBuffer->insertPoint(point2D * IMAGE_SCALE[EnvImage::RGB_THERMAL_IMAGE], point3D, pointIndex);
		points2D[pointIndex] = point2D;
	}
	
	DepthBuffer* visiblePoints = depthBuffer->getVisiblePoints();	
	for (auto& visiblePoint : *visiblePoints)
	{
		if (camera->getImage()->retrieveColor(EnvImage::THERMAL_IMAGE, points2D[visiblePoint.second.second], color))
		{
			colorData->_color[EnvPointCloud::THERMAL][visiblePoint.second.second] += vec4(color, 1.0f);
			++colorData->_numContributions[EnvPointCloud::THERMAL][visiblePoint.second.second];

#if COMPUTE_ERROR
			this->insertPointCameraHashMap(*pointCamera, visiblePoint.second.second, camera);
#endif
			
			if (camera->getImage()->getTemperature(points2D[visiblePoint.second.second], temperature))
			{
				colorData->_color[EnvPointCloud::TEMPERATURE][visiblePoint.second.second] += vec4(vec3(temperature), 1.0f);
				++colorData->_numContributions[EnvPointCloud::TEMPERATURE][visiblePoint.second.second];
			}
		}
	}

	//depthBuffer->saveImage(camera->getImage()->getName() + ".png");
}