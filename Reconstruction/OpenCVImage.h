#pragma once

#include "Geometry/2D/Rectangle2D.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

/**
*	@file CameraModel.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 04/05/2021
*/

#define JPG_EXTENSION ".jpg"
#define TIF_EXTENSION ".tif"

/**
*	@brief
*/
class OpenCVImage
{
protected:
	cv::Mat		_image;				//!< OpenCV image

public:
	/**
	*	@brief
	*/
	OpenCVImage(const std::string& filename, int flags = cv::IMREAD_GRAYSCALE);

	/**
	*	@brief 
	*/
	OpenCVImage(cv::Mat& image);

	/**
	*	@brief 
	*/
	OpenCVImage(const std::vector<std::vector<unsigned>>& values);

	// OpenCV operations

	/**
	*	@brief 
	*/
	OpenCVImage* blur(unsigned kernelSize);

	/**
	*	@brief 
	*/
	OpenCVImage* convertToGrayscale();

	/**
	*	@brief 
	*/
	OpenCVImage* crop(cv::Rect& rectangle);

	/**
	*	@brief 
	*/
	OpenCVImage* drawRectangle(Rectangle2D* rectangle);

	/**
	*	@brief 
	*/
	bool enhancedCorrelationCoefficient(OpenCVImage* image, const int motionModel, const unsigned numIterations, const float precisionFactor, cv::Mat& alignmentMatrix);

	/**
	*	@brief 
	*/
	OpenCVImage* resize(const ivec2& size);

	/**
	*	@brief 
	*/
	bool saveImage(const std::string& filename);

	/**
	*	@brief 
	*/
	OpenCVImage* undistort(const mat3& matrixK, const vec3& radialDistortion, const vec2& tangentialDistortion, Rectangle2D& cropRectangle);

	// Getters

	/**
	*	@return 
	*/
	vec3 getColor(const ivec2& pixel);

	/**
	*	@return
	*/
	vec3 getColor(const vec2& pixel);

	/**
	*	@return  
	*/
	float getColor(const unsigned row, const unsigned col);

	/**
	*	@return Size of image.
	*/
	ivec2 getSize() { return ivec2(_image.cols, _image.rows); }

	/**
	*	@return  
	*/
	bool isEmpty() { return _image.empty(); }

public:
	/**
	*	@brief 
	*/
	static Rectangle2D* composeRectangle(Rectangle2D* rectangle, cv::Mat& warpMatrix);
};

