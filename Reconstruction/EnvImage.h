#pragma once

#include "Geometry/2D/Rectangle2D.h"
#include "Reconstruction/OpenCVImage.h"

/**
*	@file EnvImage.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 01/02/2021
*/

#define BINARY_EXTENSION ".bin"
#define CSV_DELIMITER ','
#define CSV_EXTENSION ".csv"
#define NUM_FILENAME_CHARS 20
#define NUM_IMAGE_TYPES 3
#define PNG_EXTENSION ".png"
#define RECTANGLE_OFFSET 1.0f
#define RECTANGLE_THRESHOLD vec2(20.0f, 20.0f)
#define TEMPERATURE_EXTENSION "-temperature"

#define NUM_ECC_ITERATIONS 60
#define ECC_PRECISION 1e-1

/**
*	@brief
*/
class EnvImage
{
public:
	enum ImageType : uint8_t
	{
		RGB_IMAGE = 0, RGB_THERMAL_IMAGE = 1, THERMAL_IMAGE = 2
	};

	/**
	*	@brief 
	*/
	static std::string imageToString(const ImageType imageType)
	{
		switch (imageType)
		{
		case RGB_IMAGE:				return "RGB";
		case RGB_THERMAL_IMAGE:		return "RGBThermal";
		case THERMAL_IMAGE:			return "Thermal";
		default:					return "Unknown";
		}
	}

	/**
	*	@brief 
	*/
	struct ImageNameChar
	{
		char _filename[30];					//!<

		/**
		*	@return 
		*/
		void fromString(std::string& imageName) { std::strcpy(_filename, imageName.c_str()); }

		/**
		*	@return 
		*/
		std::string toString() { return std::string(_filename); }
	};

protected:
	const static std::string CSV_ROOT_FOLDER;									//!<
	const static std::string IMAGES_ROOT_FOLDER;								//!<
	const static std::string RESULT_ECC_FOLDER;									//!< 
	const static std::string ROOT_NAME[NUM_IMAGE_TYPES];						//!<

	const static std::string BINARY_FILES_FOLDER;								//!<

protected:
	static std::unordered_map<std::string, EnvImage*>	_globalImage[NUM_IMAGE_TYPES];				//!<
	static unsigned										_globalIdentifier[NUM_IMAGE_TYPES];			//!<
	static Rectangle2D									_globalRectangle[NUM_IMAGE_TYPES];
	static vec2											_maxMinTemperature;

protected:
	ImageType			_imageType;							//!< 

	// Image core
	OpenCVImage*		_image;								//!< 
		
	// Metadata
	unsigned			_id;								//!<
	std::string			_filename;							//!<
	ivec2				_size;								//!< 
	std::vector<float>	_temperature;						//!<

	// Dependent images
	EnvImage*			_alignedImage[NUM_IMAGE_TYPES];		//!<
	mat3				_warpMatrix[NUM_IMAGE_TYPES];		//!<
	bool				_registered[NUM_IMAGE_TYPES];		//!<

	// Other properties
	mat3				_matrixK;								//!<
	vec3				_radialDistortion;						//!<
	vec2				_tangentialDistortion;					//!<

public:
	/**
	*	@return 
	*/
	static float getMaximumTemperature() { return _maxMinTemperature.x; }

	/**
	*	@return
	*/
	static float getMinimumTemperature() { return _maxMinTemperature.y; }

protected:
	/**
	*	@brief
	*/
	static std::string getPairString(const EnvImage::ImageType image1, const EnvImage::ImageType image2);

public:
	/**
	*	@brief 
	*/
	static void buildRectangles();

	/**
	*	@brief
	*/
	static bool loadAlignmentMatrices(const std::string& projectName, const EnvImage::ImageType imageIdx_01, const EnvImage::ImageType imageIdx_02);

	/**
	*	@brief
	*/
	static bool loadBinaryTemperatureData(const std::string& projectName);

	/**
	*	@brief 
	*/
	static void loadImages(const std::string& projectName, const EnvImage::ImageType imageType, int loadFlags = cv::IMREAD_GRAYSCALE);

	/**
	*	@brief
	*/
	static void loadTemperatureData(const std::string& projectName);

	/**
	*	@brief
	*/
	static bool saveAlignmentRectangle(const std::string& projectName, const EnvImage::ImageType imageIdx_01, const EnvImage::ImageType imageIdx_02, bool overwrite = false);
	
	/**
	*	@brief 
	*/
	static bool saveTemperatureData(const std::string& projectName, bool overwrite = false);

	/**
	*	@brief 
	*/
	static void updateTemperatureBoundaries(const float temperature);

protected:
	/**
	*	@return 
	*/
	std::string getFilenameWithoutExtension();

	/**
	*	@return 
	*/
	unsigned getTemperatureIndex(const ivec2& point);

	/**
	*	@return
	*/
	float getTemperatureValue(const vec2& point);

	/**
	*	@return
	*/
	float getTemperatureValue(const ivec2& point);

	/**
	*	@brief 
	*/
	void loadImage(const std::string& projectName, int loadFlags = cv::IMREAD_GRAYSCALE);

	/**
	*	@brief 
	*/
	bool readTemperature(const std::string& projectName);

	/**
	*	@brief 
	*/
	void registerImage();
	
	/**
	*	@brief 
	*/
	bool registerImage(EnvImage* image, mat3& glmWarpMatrix, bool saveGraphicResult);

	/**
	*	@brief 
	*/
	vec2 transformPoint(EnvImage::ImageType imageType, const vec2& point);

public:
	/**
	*	@brief 
	*/
	EnvImage(const std::string& filename, ImageType imageType);

	/**
	*	@brief Destructor.
	*/
	~EnvImage();

	/**
	*	@brief Destructor.
	*/
	EnvImage(const EnvImage& image) = delete;

	/**
	*	@brief Destructor.
	*/
	EnvImage& operator=(const EnvImage& image) = delete;

	/**
	*	@brief
	*/
	void linkImage(const EnvImage::ImageType imageType, EnvImage* image) { _alignedImage[imageType] = image; }

	/**
	*	@brief
	*	@return 
	*/
	bool isImageAccessible(const EnvImage::ImageType imageType, const vec2& point);

	/**
	*	@brief
	*/
	bool registerImage(const EnvImage::ImageType imageType, bool saveGraphicResult = false);

	/**
	*	@brief 
	*/
	bool retrieveColor(const EnvImage::ImageType imageType, const vec2& point, vec3& color);

	/**
	*	@return 
	*/
	vec2 transformTo2D(const vec2& point, const EnvImage::ImageType imageType);

	// Getters

	/**
	*	@return Unique identifier. 
	*/
	unsigned getID() { return _id; }

	/**
	*	@return 
	*/
	std::string getName() { return _filename; }

	/**
	*	@return True if there exist image of given type registered with this one. 
	*/
	bool isRegistered(const EnvImage::ImageType imageType) { return _registered[imageType]; }

	/**
	*	@return  
	*/
	vec2 getPrincipalPoint() { return vec2(_matrixK[2][0], _matrixK[2][1]); }

	/**
	*	@return 
	*/
	ivec2 getSize() { return _size; }

	/**
	*	@return
	*/
	bool getTemperature(const vec2& point, float& temperature);

	// Setters

	/**
	*	@brief 
	*/
	void setMatrixK(const mat3& matrixK) { _matrixK = matrixK; }

	/**
	*	@brief
	*/
	void setRadialDistortion(const vec3& distortion) { _radialDistortion = distortion; }

	/**
	*	@brief 
	*/
	void setSize(const ivec2& size);

	/**
	*	@brief 
	*/
	void setTangentialDistortion(const vec2& distortion) { _tangentialDistortion = distortion; }
};

