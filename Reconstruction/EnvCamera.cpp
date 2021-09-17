#include "stdafx.h"
#include "EnvCamera.h"

#include <glm/gtc/type_ptr.hpp>

/// Initialization of static attributes
std::unordered_map<std::string, EnvCamera*> EnvCamera::_globalCameraName[NUM_IMAGE_TYPES];
std::unordered_map<EnvCamera*, EnvCamera*> EnvCamera::_rgbThermalPair;
std::vector<EnvCamera*>	EnvCamera::_globalCamera[NUM_IMAGE_TYPES];
std::vector<vec3>			EnvCamera::_globalLocalPosition[NUM_IMAGE_TYPES];
std::vector<vec3>			EnvCamera::_globalLocalRotation[NUM_IMAGE_TYPES];

vec2						EnvCamera::_apertureAngle[NUM_IMAGE_TYPES];
float						EnvCamera::_focalLength[NUM_IMAGE_TYPES];
vec2						EnvCamera::_imageCropOffset[NUM_IMAGE_TYPES];
vec2						EnvCamera::_imageCropScale[NUM_IMAGE_TYPES];
ivec2						EnvCamera::_originalImageSize[NUM_IMAGE_TYPES];
vec2						EnvCamera::_sensorWidth[NUM_IMAGE_TYPES];
ivec2						EnvCamera::_temperatureOffset;
vec2						EnvCamera::_widthHeight_mm[NUM_IMAGE_TYPES];

/// [Public methods]

EnvCamera::EnvCamera(EnvImage::ImageType imageType, const std::string& filename)
	:	_imageName(filename), _imageType(imageType),
		_localPosition(.0f), _utmPosition(.0f), _omegaPhiKappa(.0f), _pMatrix(1.0f)
{
	EnvCamera::saveGlobalData(this);

	_image = new EnvImage(filename, imageType);
}

EnvCamera::~EnvCamera()
{
}

bool EnvCamera::isPointInside(const EnvImage::ImageType imageType, const vec2& point)
{
	if (_image)
	{
		return _image->isImageAccessible(_imageType, point);
	}

	return false;
}

vec2 EnvCamera::transformTo2D(const vec4& point)
{
	const vec4 newPoint = _pMatrix * point;
	const vec2 point2D = vec2(newPoint.x / newPoint.z, newPoint.y / newPoint.z);

	return point2D;
}

vec2 EnvCamera::transformTo2D(const vec4& point, const EnvImage::ImageType imageType)
{
	const vec4 newPoint = _pMatrix * point;
	const vec2 point2D = vec2(newPoint.x / newPoint.z, newPoint.y / newPoint.z);

	return _image->transformTo2D(point2D, imageType);
}

void EnvCamera::setLocalPosition(const vec3& position)
{
	_localPosition = position;

	EnvCamera::_globalLocalPosition[_imageType].push_back(_localPosition);
}

void EnvCamera::setImageSize(const ivec2& imageSize)
{
	_image->setSize(imageSize);
}

void EnvCamera::setMatrixK(std::vector<float>& floatTokens)
{
	mat3 matrixK = glm::make_mat3(floatTokens.data());
	matrixK = glm::transpose(matrixK);

	_image->setMatrixK(matrixK);
}

void EnvCamera::setOmegaPhiKappa(const float omega, const float phi, const float kappa)
{
	float normalizer = 2.0f * M_PI / 360.0f;

	_omegaPhiKappa = vec3(omega, phi, kappa);

	EnvCamera::_globalLocalRotation[_imageType].push_back(_omegaPhiKappa * normalizer);
}

void EnvCamera::setRotationMatrix(std::vector<float>& floatTokens)
{
	_rotationMatrix = glm::make_mat3(floatTokens.data());
	_rotationMatrix = glm::transpose(_rotationMatrix);
}

/// [Protected methods]

bool EnvCamera::registerImage(const EnvImage::ImageType imageType)
{
	return _image->registerImage(imageType);
}

/// [Static protected methods]

void EnvCamera::saveGlobalData(EnvCamera* camera)
{
	_globalCameraName[camera->_imageType][camera->_imageName] = camera;
	_globalCamera[camera->_imageType].push_back(camera);
}

/// [Static public methods]

void EnvCamera::calculateApertureAngle(const EnvImage::ImageType imageType)
{
	_apertureAngle[imageType].x = std::atan(_sensorWidth[imageType].x / _focalLength[imageType]);
	_apertureAngle[imageType].y = std::atan(_sensorWidth[imageType].y / _focalLength[imageType]);
}

EnvCamera* EnvCamera::getCamera(const EnvImage::ImageType imageType, const std::string& name)
{
	auto cameraIt = _globalCameraName[imageType].find(name);

	if (cameraIt != _globalCameraName[imageType].end())
	{
		return cameraIt->second;
	}

	return nullptr;
}

float EnvCamera::getGSD(const EnvImage::ImageType imageType, const float heightMeters)
{
	// x => width, could also be performed through height
	// 100 => dm => cm / pixel
	return (_widthHeight_mm[imageType].x * heightMeters * 100.0f) / (_focalLength[imageType] * _originalImageSize[imageType].x);
}

void EnvCamera::linkImages(const EnvImage::ImageType imageType_01, const EnvImage::ImageType imageType_02)
{
	if (imageType_01 == EnvImage::RGB_THERMAL_IMAGE && imageType_02 == EnvImage::THERMAL_IMAGE)
	{
		const unsigned numCameras = _globalCamera[imageType_01].size();
		std::string cameraName, numberStr, root;
		int numberInt;
		EnvCamera* thermalCamera;

		for (int camera = 0; camera < numCameras; ++camera)
		{
			cameraName = _globalCamera[imageType_01][camera]->getImage()->getName();
			const size_t dotIndex = cameraName.find_last_of(".");
			root = cameraName.substr(0, dotIndex - 4);
			numberStr = cameraName.substr(dotIndex - 4, 4);
			numberInt = std::stoi(numberStr) - 1;
			numberStr = std::to_string(numberInt);
			numberStr = std::string(4 - numberStr.size(), '0').append(numberStr).append("_R").append(TIF_EXTENSION);
			thermalCamera = EnvCamera::getCamera(imageType_02, root + numberStr);

			if (thermalCamera)
			{
				_globalCamera[imageType_01][camera]->getImage()->linkImage(imageType_02, thermalCamera->getImage());

				// Link both cameras
				_rgbThermalPair[_globalCamera[imageType_01][camera]] = thermalCamera;
			}
		}
	}
}

void EnvCamera::registerImages(const EnvImage::ImageType imageType_01, const EnvImage::ImageType imageType_02)
{
	if (imageType_01 == EnvImage::RGB_THERMAL_IMAGE && imageType_02 == EnvImage::THERMAL_IMAGE)
	{
		const unsigned numCameras = _globalCamera[imageType_01].size();
		unsigned registeredImages = 0, numImages = numCameras;

		for (int camera = 0; camera < numCameras; ++camera)
		{
 			registeredImages += unsigned(_globalCamera[imageType_01][camera]->registerImage(imageType_02));
		}

		std::cout << "Number of Registered Images: " << registeredImages << " from " << numImages << "; " << float(registeredImages) / numImages * 100.0f << "%" << std::endl;
	}
}

void EnvCamera::setWidthHeightRelation(const EnvImage::ImageType imageType, const vec2& mm, const vec2& pixel)
{
	_widthHeight_mm[imageType] = vec2(_originalImageSize[imageType]) * mm / pixel;
}