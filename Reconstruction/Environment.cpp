#include "stdafx.h"
#include "Environment.h"

#include <fstream>
#include <filesystem>
#include <regex>

#include "Graphics/Application/MaterialList.h"
#include "Graphics/Core/ShaderList.h"
#include "Geometry/3D/AABB.h"
#include "Reconstruction/DepthBufferAugmentation.h"
#include "Reconstruction/NaiveThermalAugmentation.h"
#include "Reconstruction/OcclusionAugmentation.h"
#include "Utilities/ChronoUtilities.h"
#include "Utilities/FileManagement.h"

/// Initialization of static methods
const std::string Environment::SCENE_ROOT_FOLDER = "Pix4D/";

const std::string Environment::IMAGE_TYPE_FOLDER[] = { "RGB/", "RGBThermal/", "Thermal/" };

const std::string Environment::ROOT_NAME[] = { "RGBPointCloud", "RGBThermalPointCloud", "ThermalPointCloud" };

const std::string Environment::CAMERA_FOLDER = "Camera/";
const std::string Environment::CALIBRATED_CAMERA_PARAMETERS = "_calibrated_camera_parameters.txt";
const std::string Environment::CALIBRATED_EXTERNAL_PARAMETERS = "_calibrated_external_camera_parameters.txt";
const std::string Environment::CALIBRATED_INTERNAL_PARAMETERS = "_calibrated_internal_camera_parameters.cam";
const std::string Environment::CALIBRATED_PIX4D_INTERNAL_PARAMETERS = "_pix4d_calibrated_internal_camera_parameters.cam";
const std::string Environment::CALIBRATED_IMAGE_POSITION = "_calibrated_images_position.txt";
const std::string Environment::IMAGE_CROP_PARAMS = "_image_crop.txt";
const std::string Environment::OFFSET = "_offset.xyz";
const std::string Environment::PMATRIX = "_pmatrix.txt";
const std::string Environment::TEMPERATURE_OFFSET = "_temperature_offset.txt";

const std::string Environment::POINT_CLOUD_FOLDER = "PointCloud/";
const std::string Environment::POINT_CLOUD_FILE[] = { "RGBPointCloud", "RGBThermalPointCloud_02", "ThermalPointCloud" };

const std::string Environment::STUDY_AREA_FOLDER = "StudyArea";

const std::vector<std::unique_ptr<ThermalAugmentation>> Environment::THERMAL_AUGMENTATION_APPLICATOR = Environment::getThermalAugmentationApplicators();

/// Public methods

Environment::Environment(const std::string& projectName) :
	_projectName{projectName}, _thermalAugmentationType{ThermalAugmentationType::NAIVE}
{
	for (int i = 0; i < NUM_IMAGE_TYPES; ++i)
	{
		_pointCloud[i]			= nullptr;
		_octree[i]				= nullptr;

		_pointCloudBVH[i]		= nullptr;
		_radiusOctree[i]		= nullptr;
		_drawOctree[i]			= nullptr;
		_drawRadiusOctree[i]	= nullptr;

		_classifier[i]			= nullptr;
	}
}

Environment::~Environment()
{
	for (int image = 0; image < NUM_IMAGE_TYPES; ++image)
	{
		for (std::pair<std::string, EnvCamera*> cameraPair : _camera[image]) delete cameraPair.second;

		delete _pointCloud[image];
		delete _octree[image];

		delete _pointCloudBVH[image];
		delete _radiusOctree[image];
		delete _drawOctree[image];
		delete _drawRadiusOctree[image];

		delete _classifier[image];
	}
}

ThermalAugmentation::VolatileAugmentationData* Environment::augmentate3DPoints()
{
	std::cout << "Augmenting RGB point cloud..." << std::endl;
	ChronoUtilities::initChrono();
	
	auto colorData = THERMAL_AUGMENTATION_APPLICATOR[_thermalAugmentationType]->augmentatePointCloud(_pointCloud[EnvImage::RGB_THERMAL_IMAGE], _radiusOctree[EnvImage::RGB_THERMAL_IMAGE]);

	std::cout << ChronoUtilities::getDuration() << std::endl;
	
	return colorData;
}

void Environment::load()
{
	this->readMetadata(EnvImage::RGB_THERMAL_IMAGE);
	this->readMetadata(EnvImage::THERMAL_IMAGE);
	this->readPointClouds(EnvImage::RGB_THERMAL_IMAGE);
	this->registerImages();

	if (!_pointCloud[EnvImage::RGB_THERMAL_IMAGE]->loadSecondaryColorsFromBinary())
	{
		ThermalAugmentation::VolatileAugmentationData* colorData = this->augmentate3DPoints();
		_pointCloud[EnvImage::RGB_THERMAL_IMAGE]->setSecondaryColor(EnvPointCloud::PointColorTypes::THERMAL, colorData->_color[EnvPointCloud::THERMAL]);
		_pointCloud[EnvImage::RGB_THERMAL_IMAGE]->setSecondaryColor(EnvPointCloud::PointColorTypes::TEMPERATURE, colorData->_color[EnvPointCloud::TEMPERATURE]);
		_pointCloud[EnvImage::RGB_THERMAL_IMAGE]->writeSecondaryColorsToBinary();

		delete colorData;
	}

	this->classifyPointClouds();
	_pointCloud[EnvImage::RGB_THERMAL_IMAGE]->computeQuartiles();
}

/// Protected methods

EnvCamera* Environment::getCamera(const std::string& imageName, const EnvImage::ImageType imageType)
{
	auto cameraIt = _camera[imageType].find(imageName);
	EnvCamera* camera = nullptr;

	if (cameraIt == _camera[imageType].end())
	{
		camera = new EnvCamera(imageType, imageName);
		_camera[imageType][imageName] = camera;
	}
	else
	{
		camera = cameraIt->second;
	}

	return camera;
}

std::vector<std::unique_ptr<ThermalAugmentation>> Environment::getThermalAugmentationApplicators()
{
	std::vector<std::unique_ptr<ThermalAugmentation>> applicator(OCCLUSION + 1);

	applicator[ThermalAugmentationType::NAIVE].reset(new NaiveThermalAugmentation());
	applicator[ThermalAugmentationType::DEPTH_BUFFER_VISIBILITY].reset(new DepthBufferAugmentation());
	applicator[ThermalAugmentationType::OCCLUSION].reset(new OcclusionAugmentation());

	return applicator;
}

void Environment::readCameraCalibratedParameters(const std::string& filename, const EnvImage::ImageType imageType)
{
	std::string line;
	std::ifstream in;
	std::vector<std::string> stringTokens;
	std::vector<float> floatTokens;
	unsigned index = -1, translationOffset = 0, numberOfLines;
	bool foundImageHeader = false, foundTranslationHeader = false;
	std::vector<std::string> lines;

	// Camera temporal variables
	EnvCamera* camera = nullptr;
	std::string imageName;

	in.open(filename);
	if (!in)
	{
		std::cout << "Could not read camera calibrated parameters for " << EnvImage::imageToString(imageType) << std::endl;
		return;
	}

	while (!(in >> std::ws).eof() && !foundImageHeader)
	{
		std::getline(in, line);
		lines.push_back(line);
	}

	while (++index < lines.size() && !foundImageHeader)
	{
		const size_t xIndex = lines[index].find_last_of("x");
	
		try
		{
			numberOfLines = (xIndex != std::string::npos) ? std::stoi(std::string(1, lines[index][xIndex + 1])) : 1;
		}
		catch (std::exception& exception)
		{
			numberOfLines = 1;
		}

		if (!foundTranslationHeader)
		{
			if (lines[index].find(CAMERA_TRANSLATION_HEADER) != std::string::npos)
			{
				foundTranslationHeader = true;
			}
			else
			{
				translationOffset += numberOfLines;
			}
		}

		foundImageHeader = lines[index].find("DJI") != std::string::npos || lines[index].find("dji") != std::string::npos;
	}

	--index;			// Go back to DJI line

	while (index < lines.size())
	{
		FileManagement::clearTokens(stringTokens, floatTokens);
		FileManagement::readTokens(lines[index++], DELIMITER, stringTokens, floatTokens);

		_camera[imageType][stringTokens[0]] = camera = new EnvCamera(imageType, stringTokens[0]);
		
		camera->setImageSize(ivec2(floatTokens[0], floatTokens[1]));
		EnvCamera::setOriginalImageSize(imageType, ivec2(floatTokens[0], floatTokens[1]));

		// Camera matrix K
		FileManagement::clearTokens(stringTokens, floatTokens);
		FileManagement::readTokens(lines[index++], DELIMITER, stringTokens, floatTokens);
		FileManagement::readTokens(lines[index++], DELIMITER, stringTokens, floatTokens);
		FileManagement::readTokens(lines[index++], DELIMITER, stringTokens, floatTokens);

		camera->setMatrixK(floatTokens);

		// Radial distortion
		FileManagement::clearTokens(stringTokens, floatTokens);
		FileManagement::readTokens(lines[index++], DELIMITER, stringTokens, floatTokens);

		camera->setRadialDistortion(vec3(floatTokens[0], floatTokens[1], floatTokens[2]));

		// Tangential distortion
		FileManagement::clearTokens(stringTokens, floatTokens);
		FileManagement::readTokens(lines[index++], DELIMITER, stringTokens, floatTokens);

		camera->setTangentialDistortion(vec2(floatTokens[0], floatTokens[1]));

		// Translation
		FileManagement::clearTokens(stringTokens, floatTokens);
		FileManagement::readTokens(lines[index++], DELIMITER, stringTokens, floatTokens);

		camera->setLocalPosition(vec3(floatTokens[0], floatTokens[1], floatTokens[2]));

		// Matrix
		FileManagement::clearTokens(stringTokens, floatTokens);
		FileManagement::readTokens(lines[index++], DELIMITER, stringTokens, floatTokens);
		FileManagement::readTokens(lines[index++], DELIMITER, stringTokens, floatTokens);
		FileManagement::readTokens(lines[index++], DELIMITER, stringTokens, floatTokens);

		camera->setRotationMatrix(floatTokens);
	}
}

void Environment::readCameraCalibratedImagesPosition(const std::string& filename, const EnvImage::ImageType imageType)
{
	std::string line;
	std::ifstream in;
	std::vector<std::string> stringTokens;
	std::vector<float> floatTokens;

	in.open(filename);
	if (!in)
	{
		std::cout << "Could not read UTM positions for " << EnvImage::imageToString(imageType) << std::endl;
		return;
	}

	while (!(in >> std::ws).eof())
	{
		std::getline(in, line);

		FileManagement::clearTokens(stringTokens, floatTokens);
		FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);
	}

	in.close();
}

void Environment::readCameraExternalCalibratedParameters(const std::string& filename, const EnvImage::ImageType imageType)
{
	std::string line;
	std::ifstream in;
	std::vector<std::string> stringTokens;
	std::vector<float> floatTokens;
	EnvCamera* camera = nullptr;

	in.open(filename);
	if (!in)
	{
		std::cout << "Could not read external calibrated parameters for " << EnvImage::imageToString(imageType) << std::endl;
		return;
	}

	while (!(in >> std::ws).eof())
	{
		std::getline(in, line);

		FileManagement::clearTokens(stringTokens, floatTokens);
		FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);

		if (floatTokens.size())
		{
			camera = this->getCamera(stringTokens[0], imageType);
			camera->setUTMPosition(vec3(floatTokens[0], floatTokens[1], floatTokens[2]));
			camera->setOmegaPhiKappa(floatTokens[3], floatTokens[4], floatTokens[5]);
		}
	}

	in.close();
}

void Environment::readCameraInternalCalibratedParameters(const std::string& filename, const EnvImage::ImageType imageType)
{
	std::string line;
	std::ifstream in;
	std::vector<std::string> stringTokens;
	std::vector<float> floatTokens;

	vec2 offsetMM(.0f), offsetPixel(.0f);

	in.open(filename);
	if (!in)
	{
		std::cout << "Could not read internal calibrated parameters for " << EnvImage::imageToString(imageType) << std::endl;
		return;
	}

	while (!(in >> std::ws).eof())
	{
		std::getline(in, line);

		if (line.find(CAMERA_FOCAL_LENGTH) != std::string::npos)
		{
			FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);

			EnvCamera::setFocalLength(imageType, floatTokens[0]);

			FileManagement::clearTokens(stringTokens, floatTokens);
		}
		else if (line.find(CAMERA_SENSOR_WIDTH) != std::string::npos)
		{
			FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);

			EnvCamera::setSensorWidth(imageType, vec2(floatTokens[0], floatTokens[1]));

			FileManagement::clearTokens(stringTokens, floatTokens);
		}
		else if (line.find(CAMERA_OFFSET_MM) != std::string::npos)
		{
			std::getline(in, line);
			FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);		// X

			std::getline(in, line);
			FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);		// Y

			offsetMM = vec2(std::abs(floatTokens[0]), std::abs(floatTokens[1]));

			FileManagement::clearTokens(stringTokens, floatTokens);
		}
		else if (line.find(CAMERA_OFFSET_PIXEL) != std::string::npos)
		{
			std::getline(in, line);
			FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);		// X

			std::getline(in, line);
			FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);		// Y

			offsetPixel = vec2(std::abs(floatTokens[0]), std::abs(floatTokens[1]));

			// Last needed line => compute everything here
			EnvCamera::setWidthHeightRelation(imageType, offsetMM, offsetPixel);

			break;
		}
	}

	in.close();
}

void Environment::readImageCropArea(const std::string& filename, const EnvImage::ImageType imageType)
{
	std::string line;
	std::ifstream in;
	std::vector<std::string> stringTokens;
	std::vector<float> floatTokens;

	in.open(filename);
	if (!in)
	{
		// Image offset is not needed for every type of image, therefore we don't emit a message when this file is not found
		return;
	}

	while (!(in >> std::ws).eof())
	{
		std::getline(in, line);

		if (line.find(IMAGE_CROP_OFFSET) != std::string::npos)
		{
			FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);

			EnvCamera::setImageCropOffset(imageType, vec2(floatTokens[0], floatTokens[1]));

			FileManagement::clearTokens(stringTokens, floatTokens);
		}
		else if (line.find(IMAGE_CROP_SCALE) != std::string::npos)
		{
			FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);

			EnvCamera::setImageCropScale(imageType, vec2(floatTokens[0], floatTokens[1]));

			FileManagement::clearTokens(stringTokens, floatTokens);
		}
	}

	in.close();
}

void Environment::readMetadata(EnvImage::ImageType imageType)
{
	const std::string rootName = SCENE_ROOT_FOLDER + CAMERA_FOLDER + _projectName + "/" + IMAGE_TYPE_FOLDER[imageType] + ROOT_NAME[imageType];
	const std::string calibratedParametersFilename = rootName + CALIBRATED_CAMERA_PARAMETERS;
	const std::string externalCalibratedParametersFilename = rootName + CALIBRATED_EXTERNAL_PARAMETERS;
	const std::string imageOffsetFilename = rootName + IMAGE_CROP_PARAMS;
	const std::string internalCalibratedParametersFilename = rootName + CALIBRATED_INTERNAL_PARAMETERS;
	const std::string offsetFilename = rootName + OFFSET;
	const std::string pMatricesFilename = rootName + PMATRIX;
	const std::string temperatureOffsetFilename = rootName + TEMPERATURE_OFFSET;

	this->readCameraCalibratedParameters(calibratedParametersFilename, imageType);
	this->readOffset(offsetFilename, imageType);
	this->readPMatrices(pMatricesFilename, imageType);
	this->readCameraExternalCalibratedParameters(externalCalibratedParametersFilename, imageType);
	this->readCameraInternalCalibratedParameters(internalCalibratedParametersFilename, imageType);
	this->readImageCropArea(imageOffsetFilename, imageType);
	this->readTemperatureOffset(temperatureOffsetFilename);

	EnvCamera::calculateApertureAngle(imageType);
}

void Environment::readOffset(const std::string& filename, const EnvImage::ImageType imageType)
{
	std::string line;
	std::ifstream in;
	std::vector<std::string> stringTokens;
	std::vector<float> floatTokens;

	in.open(filename);
	if (!in)
	{
		std::cout << "Could not read offset for " << EnvImage::imageToString(imageType) << std::endl;
		return;
	}

	while (!(in >> std::ws).eof())
	{
		std::getline(in, line);

		FileManagement::clearTokens(stringTokens, floatTokens);
		FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);
	}

	if (floatTokens.size() == 3)
	{
		_offset[imageType] = vec3(floatTokens[0], floatTokens[2], floatTokens[1]);		// Y => Z
	}

	in.close();
}

void Environment::readPMatrices(const std::string& filename, const EnvImage::ImageType imageType)
{
	std::string line;
	std::ifstream in;
	std::vector<std::string> stringTokens;
	std::vector<float> floatTokens;
	EnvCamera* camera = nullptr;

	in.open(filename);
	if (!in)
	{
		std::cout << "Could not read pmatrices for " << EnvImage::imageToString(imageType) << std::endl;
		return;
	}

	while (!(in >> std::ws).eof())
	{
		std::getline(in, line);

		FileManagement::clearTokens(stringTokens, floatTokens);
		FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);

		camera = this->getCamera(stringTokens[0], imageType);
		camera->setPMatrix(mat4(floatTokens[0], floatTokens[1], floatTokens[2], floatTokens[3],
								floatTokens[4], floatTokens[5], floatTokens[6], floatTokens[7],
								floatTokens[8], floatTokens[9], floatTokens[10], floatTokens[11],
								.0f, .0f, .0f, 1.0f));
	}

	in.close();
}

void Environment::readPointClouds(const EnvImage::ImageType imageType)
{
	const std::string pointCloudPath = SCENE_ROOT_FOLDER + POINT_CLOUD_FOLDER + _projectName + "/" + POINT_CLOUD_FILE[imageType];

	_pointCloud[imageType] = new EnvPointCloud(pointCloudPath, true);
	_pointCloud[imageType]->load();
	
	_radiusOctree[imageType] = new RadiusOctree(_pointCloud[imageType]);
	//_drawRadiusOctree[imageType] = new DrawRadiusOctree(_radiusOctree[imageType]);
	//_drawRadiusOctree[imageType]->load();

	//_octree[imageType] = new OctreePointCloud(_pointCloud[imageType]);
	//_drawOctree[imageType] = new DrawOctreePointCloud(_octree[imageType]);
	//_drawOctree[imageType]->load();
}

void Environment::readTemperatureOffset(const std::string& filename)
{
	std::string line;
	std::ifstream in;
	std::vector<std::string> stringTokens;
	std::vector<float> floatTokens;

	in.open(filename);
	if (!in)
	{
		// Image offset is not needed for every type of image, therefore we don't emit a message when this file is not found
		return;
	}

	while (!(in >> std::ws).eof())
	{
		std::getline(in, line);

		if (line.find(IMAGE_OFFSET) != std::string::npos)
		{
			FileManagement::readTokens(line, DELIMITER, stringTokens, floatTokens);

			EnvCamera::setTemperatureOffset(ivec2(floatTokens[0], floatTokens[1]));

			FileManagement::clearTokens(stringTokens, floatTokens);
		}
	}

	in.close();
}

/// [Non-reading related]

void Environment::classifyPointClouds()
{
	_classifier[EnvImage::RGB_THERMAL_IMAGE] = new PointCloudClassifier(_pointCloud[EnvImage::RGB_THERMAL_IMAGE]);
	_classifier[EnvImage::RGB_THERMAL_IMAGE]->load();
}

void Environment::registerImages()
{
	EnvImage::loadImages(_projectName, EnvImage::THERMAL_IMAGE, cv::IMREAD_COLOR);
	EnvCamera::linkImages(EnvImage::RGB_THERMAL_IMAGE, EnvImage::THERMAL_IMAGE);
	EnvImage::buildRectangles();

	if (!EnvImage::loadAlignmentMatrices(_projectName, EnvImage::RGB_THERMAL_IMAGE, EnvImage::THERMAL_IMAGE))
	{
		EnvImage::loadImages(_projectName, EnvImage::RGB_THERMAL_IMAGE);
		EnvCamera::registerImages(EnvImage::RGB_THERMAL_IMAGE, EnvImage::THERMAL_IMAGE);
		EnvImage::saveAlignmentRectangle(_projectName, EnvImage::RGB_THERMAL_IMAGE, EnvImage::THERMAL_IMAGE);
	}

	if (!EnvImage::loadBinaryTemperatureData(_projectName))
	{
		EnvImage::loadTemperatureData(_projectName);
		EnvImage::saveTemperatureData(_projectName);
	}
}