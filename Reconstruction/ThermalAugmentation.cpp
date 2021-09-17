#include "stdafx.h"
#include "ThermalAugmentation.h"

#include "Utilities/ChronoUtilities.h"
#include "Utilities/Histogram.h"

// [Initialization of static methods]


const float		ThermalAugmentation::DISTANCE_DIVISOR = 30.0f;
const float		ThermalAugmentation::IMAGE_SCALE[] = { 1.0f, 1.0f / 2.0f, 1.0f };
const unsigned	ThermalAugmentation::SEARCH_RADIUS = 20.0f;

// [Protected methods]

ThermalAugmentation::VolatileAugmentationData* ThermalAugmentation::getColorData(EnvPointCloud* pointCloud)
{
	VolatileAugmentationData* colorData = new VolatileAugmentationData;

	// Resize color buffers to point cloud size
	colorData->calloc(EnvPointCloud::THERMAL, pointCloud->getNumberOfPoints());
	colorData->calloc(EnvPointCloud::TEMPERATURE, pointCloud->getNumberOfPoints());

	return colorData;
}

void ThermalAugmentation::computeErrors(ThermalAugmentation::VolatileAugmentationData* colorData, std::vector<vec4>* points, std::unordered_map<GLuint, std::vector<EnvCamera*>>& pointCamera)
{
	float rootMeanSquaredError = .0f, meanAbsoluteError = .0f, standardDeviation = .0f,
		rootMeanSquaredErrorMean = .0f, rootMeanSquaredErrorCamera, meanAbsoluteErrorMean = .0f, meanAbsoluteErrorCamera, standardDeviationMean = .0f, standardDeviationCamera,
		mean, pointColor, error;
	vec2 imagePoint;
	vec3 thermalColor;
	unsigned N = 0, maxNumberSamples = 0, indexBRDF = 0;

	for (auto& hashMapIt : pointCamera)
	{
		std::vector<float> colors;
		pointColor = colorData->_color[EnvPointCloud::THERMAL][hashMapIt.first].x * 255.0f;
		mean = rootMeanSquaredErrorCamera = meanAbsoluteErrorCamera = standardDeviationCamera = .0f;

#if SAVE_BRDF_POINT
		if (hashMapIt.second.size() > maxNumberSamples)
		{
			maxNumberSamples = hashMapIt.second.size();
			indexBRDF = hashMapIt.first;
		}
#endif

		for (EnvCamera* camera : hashMapIt.second)
		{
			vec2 imagePoint = camera->transformTo2D(points->at(hashMapIt.first));
			imagePoint = ivec2(std::floor(imagePoint.x), std::floor(imagePoint.y));
			camera->getImage()->retrieveColor(EnvImage::THERMAL_IMAGE, imagePoint, thermalColor);
			colors.push_back(thermalColor.x);

			error = pointColor - thermalColor.x;
			rootMeanSquaredError += error * error;
			rootMeanSquaredErrorCamera += error * error;
			meanAbsoluteError += std::abs(error);
			meanAbsoluteErrorCamera += std::abs(error);
			mean += thermalColor.x;

			++N;
		}

		mean /= hashMapIt.second.size();

		for (float value : colors)
		{
			standardDeviation += (value - mean) * (value - mean);
			standardDeviationCamera += (value - mean) * (value - mean);
		}

		standardDeviationMean += std::sqrt(standardDeviationCamera / hashMapIt.second.size());
		rootMeanSquaredErrorMean += sqrt(rootMeanSquaredErrorCamera / hashMapIt.second.size());
		meanAbsoluteErrorMean += meanAbsoluteErrorCamera / hashMapIt.second.size();
	}

	rootMeanSquaredError = sqrt(rootMeanSquaredError / N);
	meanAbsoluteError = meanAbsoluteError / N;
	standardDeviation = sqrt(standardDeviation / N);

	rootMeanSquaredErrorMean /= pointCamera.size();
	meanAbsoluteErrorMean /= pointCamera.size();
	standardDeviationMean /= pointCamera.size();

	std::cout << "Root Mean Square Error: " << rootMeanSquaredError << std::endl;
	std::cout << "Root Mean Square Error per Point: " << rootMeanSquaredErrorMean << std::endl;
	std::cout << "Mean Absolute Error: " << meanAbsoluteError << std::endl;
	std::cout << "Mean Absolute Error per Point: " << meanAbsoluteErrorMean << std::endl;
	std::cout << "Standard Deviation: " << standardDeviation << std::endl;
	std::cout << "Standard Deviation per Point: " << standardDeviationMean << std::endl;

#if SAVE_BRDF_POINT
	std::cout << "Number of samples for BRDF: " << maxNumberSamples << " for point indexed with " << indexBRDF << std::endl;
	saveBRDF(points->at(indexBRDF), &(pointCamera[indexBRDF]));
#endif
}

void ThermalAugmentation::computeHistogram(ThermalAugmentation::VolatileAugmentationData* colorData, std::vector<vec4>* points, std::unordered_map<GLuint, std::vector<EnvCamera*>>& pointCamera)
{
	std::unordered_map<GLuint, float> visiblePixels;
	std::vector<float> imageColors, cloudColors;
	unsigned pixelID;
	float pointColor;
	ivec2 imageSize;
	vec2 imagePoint;
	vec3 thermalColor;

	for (auto& hashMapIt : pointCamera)
	{
		for (EnvCamera* camera : hashMapIt.second)
		{
			imageSize = camera->getImage()->getSize();
			vec2 imagePoint = camera->transformTo2D(points->at(hashMapIt.first));
			imagePoint = ivec2(std::floor(imagePoint.x), std::floor(imagePoint.y));
			pixelID = camera->getImage()->getID() * imageSize.x * imageSize.y + imagePoint.x + imagePoint.y * imageSize.x;

			if (visiblePixels.find(pixelID) == visiblePixels.end())
			{
				camera->getImage()->retrieveColor(EnvImage::THERMAL_IMAGE, imagePoint, thermalColor);
				visiblePixels[pixelID] = thermalColor.x;
			}
		}
	}

	for (auto& imagePoint: visiblePixels)
	{
		imageColors.push_back(imagePoint.second / 255.0f);
	}

	Histogram* imageHistogram = new Histogram(&imageColors);
	imageHistogram->buildHistogram(256, .0f, 1.0f, true);
	imageHistogram->exportLatex("Histogram/ImageLatex.txt");

	for (auto& pointColor: colorData->_color[EnvPointCloud::THERMAL])
	{
		if (pointColor.w > glm::epsilon<float>()) cloudColors.push_back(pointColor.x);
	}

	Histogram* cloudHistogram = new Histogram(&cloudColors);
	cloudHistogram->buildHistogram(256, .0f, 1.0f, true);
	cloudHistogram->exportLatex("Histogram/CloudLatex.txt");

	//delete imageHistogram;
	delete cloudHistogram;
}

void ThermalAugmentation::insertPointCameraHashMap(std::unordered_map<GLuint, std::vector<EnvCamera*>>& pointCamera, GLuint index, EnvCamera* camera)
{
	auto pointCameraIt = pointCamera.find(index);
	if (pointCameraIt == pointCamera.end())
	{
		pointCamera[index] = std::vector<EnvCamera*>{ camera };
	}
	else
	{
		pointCameraIt->second.push_back(camera);
	}
}

void ThermalAugmentation::saveBRDF(const vec4& point, const std::vector<EnvCamera*>* cameras)
{
	vec3 viewDirection, color;
	float azimuth, elevation;
	std::string pgllFile = "#format: x y z [uA]\n";

	for (EnvCamera* camera : *cameras)
	{
		vec2 imagePoint = camera->transformTo2D(point);
		imagePoint = ivec2(std::floor(imagePoint.x), std::floor(imagePoint.y));
		camera->getImage()->retrieveColor(EnvImage::THERMAL_IMAGE, imagePoint, color);

		viewDirection = glm::normalize(camera->getLocalPosition() - vec3(point));
		viewDirection = vec3(viewDirection.x, viewDirection.z, viewDirection.y);
		azimuth = std::atan2(viewDirection.z, viewDirection.x);
		elevation = std::atan2(sqrt(viewDirection.x * viewDirection.x + viewDirection.z * viewDirection.z), viewDirection.y);

		pgllFile += std::to_string(viewDirection.x) + " " + std::to_string(viewDirection.y) + " " + std::to_string(viewDirection.z) + " " + std::to_string(color.x / 255.0f) + "\n";
	}

	{
		std::ofstream out("BRDF/BRDF.txt");
		out << pgllFile;
		out.close();
	}
}

// [Volatile data]

void ThermalAugmentation::VolatileAugmentationData::calloc(const EnvPointCloud::PointColorTypes colorType, const unsigned size)
{
	_color[colorType].resize(size, vec4(.0f));
	_numContributions[colorType].resize(size, 0);
}