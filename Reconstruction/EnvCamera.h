#pragma once

#include "EnvImage.h"
#include "Graphics/Core/VAO.h"

/**
*	@file EnvCamera.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 09/25/2020
*/

/**
*	@brief
*/
class EnvCamera
{
protected:
	static std::unordered_map<std::string, EnvCamera*>	_globalCameraName[NUM_IMAGE_TYPES];		//!<
	static std::unordered_map<EnvCamera*, EnvCamera*>	_rgbThermalPair;						//!<
	static std::vector<EnvCamera*>	_globalCamera[NUM_IMAGE_TYPES];								//!<
	static std::vector<vec3>			_globalLocalPosition[NUM_IMAGE_TYPES];						//!<
	static std::vector<vec3>			_globalLocalRotation[NUM_IMAGE_TYPES];						//!<

	static vec2							_apertureAngle[NUM_IMAGE_TYPES];							//!<
	static float						_focalLength[NUM_IMAGE_TYPES];								//!<
	static vec2							_imageCropOffset[NUM_IMAGE_TYPES];							//!<
	static vec2							_imageCropScale[NUM_IMAGE_TYPES];							//!<
	static ivec2						_originalImageSize[NUM_IMAGE_TYPES];						//!<
	static vec2							_sensorWidth[NUM_IMAGE_TYPES];								//!<
	static ivec2						_temperatureOffset;											//!<
	static vec2							_widthHeight_mm[NUM_IMAGE_TYPES];							//!<

protected:
	// EnvImage
	EnvImage*							_image;														//!< 
	EnvImage::ImageType				_imageType;													//!<

	// Pix4D information
	std::string							_imageName;													//!<
	vec3								_localPosition;												//!<
	vec3								_omegaPhiKappa;												//!<
	mat4								_pMatrix;													//!< Allows to transform 3D points to 2D (for undistorted images) 
	mat3								_rotationMatrix;											//!<
	vec3								_utmPosition;												//!<

protected:
	// Static methods

	/**
	*	@brief 
	*/
	static void saveGlobalData(EnvCamera* camera);

public:
	// Static methods

	/**
	*	@brief 
	*/
	static void calculateApertureAngle(const EnvImage::ImageType imageType);

	/**
	*	@return
	*/
	static EnvCamera* getCamera(const EnvImage::ImageType imageType, const std::string& name);

	/**
	*	@return
	*/
	static std::vector<EnvCamera*>* getCameras(const EnvImage::ImageType imageType) { return &_globalCamera[imageType]; }

	/**
	*	@return 
	*/
	static std::vector<vec3>* getCamerasPositions(const EnvImage::ImageType imageType) { return &_globalLocalPosition[imageType]; }

	/**
	*	@return
	*/
	static std::vector<vec3>* getCamerasRotations(const EnvImage::ImageType imageType) { return &_globalLocalRotation[imageType]; }

	/**
	*	@return 
	*/
	static float getFocalLength(const EnvImage::ImageType imageType) { return _focalLength[imageType]; }

	/**
	*	@return
	*/
	static float getGSD(const EnvImage::ImageType imageType, const float heightMeters);

	/**
	*	@return
	*/
	static vec2 getImageCropOffset(const EnvImage::ImageType imageType) { return _imageCropOffset[imageType]; }

	/**
	*	@return  
	*/
	static vec2 getImageCropScale(const EnvImage::ImageType imageType) { return _imageCropScale[imageType]; }

	/**
	*	@return  
	*/
	static vec2 getOriginalImageSize(const EnvImage::ImageType imageType) { return _originalImageSize[imageType]; }

	/**
	*	@return  
	*/
	static ivec2 getTemperatureOffset() { return _temperatureOffset; }

	/**
	*	@return 
	*/
	static EnvCamera* getThermalCamera(EnvCamera* rgbCamera) { return _rgbThermalPair[rgbCamera]; }

	/**
	*	@return 
	*/
	static vec2 getWidthHeightMM(const EnvImage::ImageType imageType) { return _widthHeight_mm[imageType]; }

	/**
	*	@brief
	*/
	static void linkImages(const EnvImage::ImageType imageType_01, const EnvImage::ImageType imageType_02);

	/**
	*	@brief
	*/
	static void registerImages(const EnvImage::ImageType imageType_01, const EnvImage::ImageType imageType_02);

	/**
	*	@brief 
	*/
	static void setFocalLength(const EnvImage::ImageType imageType, const float focalLength) { _focalLength[imageType] = focalLength; }

	/**
	*	@brief  
	*/
	static void setImageCropOffset(const EnvImage::ImageType imageType, const vec2& offset) { _imageCropOffset[imageType] = offset; }

	/**
	*	@brief  
	*/
	static void setImageCropScale(const EnvImage::ImageType imageType, const vec2& scale) { _imageCropScale[imageType] = scale; }

	/**
	*	@brief  
	*/
	static void setOriginalImageSize(const EnvImage::ImageType imageType, const ivec2& size) { _originalImageSize[imageType] = size; }

	/**
	*	@brief
	*/
	static void setSensorWidth(const EnvImage::ImageType imageType, const vec2& sensorWidth) { _sensorWidth[imageType] = sensorWidth; }

	/**
	*	@brief  
	*/
	static void setTemperatureOffset(const ivec2& offset) { _temperatureOffset = offset; }

	/**
	*	@brief
	*/
	static void setWidthHeightRelation(const EnvImage::ImageType imageType, const vec2& mm, const vec2& pixel);

protected:
	/**
	*	@brief
	*	@param imageType
	*	@return
	*/
	bool registerImage(const EnvImage::ImageType imageType);

public:
	/**
	*	@brief 
	*/
	EnvCamera(EnvImage::ImageType imageType, const std::string& filename);

	/**
	*	@brief Destructor.
	*/
	~EnvCamera();

	/**
	*	@brief Deleted copy constructor.
	*/
	EnvCamera(const EnvCamera& camera) = delete;

	/**
	*	@brief Deleted assignment operator.
	*/
	EnvCamera& operator=(const EnvCamera& camera) = delete;

	/**
	*	@return 
	*/
	bool isPointInside(const EnvImage::ImageType imageType, const vec2& point);

	/**
	*	@return 
	*/
	vec2 transformTo2D(const vec4& point);

	/**
	*	@return
	*/
	vec2 transformTo2D(const vec4& point, const EnvImage::ImageType imageType);

	// Getters

	/**
	*	@return
	*/
	EnvImage* getImage() { return _image; }

	/**
	*	@return Type of image through an enum. 
	*/
	EnvImage::ImageType getImageType() { return _imageType; }

	/**
	*	@return 
	*/
	vec3 getLocalPosition() { return _localPosition; }

	/**
	*	@return True if there exist a registered image of type 'imageType'. 
	*/
	bool isRegistered(const EnvImage::ImageType imageType) { return _image->isRegistered(imageType); }

	// Setters

	/**
	*	@brief 
	*/
	void setLocalPosition(const vec3& position);

	/**
	*	@brief 
	*/
	void setImageSize(const ivec2& imageSize);

	/**
	*	@brief
	*/
	void setMatrixK(std::vector<float>& floatTokens);

	/**
	*	@brief
	*/
	void setOmegaPhiKappa(const float omega, const float phi, const float kappa);

	/**
	*	@brief
	*/
	void setPMatrix(const mat4& pMatrix) { _pMatrix = glm::transpose(pMatrix); }

	/**
	*	@brief
	*/
	void setRadialDistortion(const vec3& distortion) { _image->setRadialDistortion(distortion); }

	/**
	*	@brief
	*/
	void setRotationMatrix(std::vector<float>& floatTokens);

	/**
	*	@brief 
	*/
	void setTangentialDistortion(const vec2& distortion) { _image->setTangentialDistortion(distortion); }

	/**
	*	@brief
	*/
	void setUTMPosition(const vec3& utmPosition) { _utmPosition = utmPosition; }
};

