#include "stdafx.h"
#include "OpenCVImage.h"

/// [Public methods]

OpenCVImage::OpenCVImage(const std::string& filename, int flags)
{
	_image = cv::imread(filename + JPG_EXTENSION, flags);

	if (_image.empty())
	{
		_image = cv::imread(filename + TIF_EXTENSION, flags);
	}

	if (_image.empty())
	{
		std::cout << "Could not read the image " << filename << std::endl;
	}
}

OpenCVImage::OpenCVImage(cv::Mat& image)
{
	_image = std::move(image);
}

OpenCVImage::OpenCVImage(const std::vector<std::vector<unsigned>>& values)
{
	if (!values.empty())
	{
		_image = cv::Mat(cv::Size(values[0].size(), values.size()), CV_8UC1);

		for (int row = 0; row < values.size(); ++row)
		{
			for (int column = 0; column < values[row].size(); ++column)
			{
				_image.at<uchar>(cv::Point(column, row)) = values[row][column];
			}
		}
	}
}

OpenCVImage* OpenCVImage::blur(unsigned kernelSize)
{
	if (kernelSize % 2 == 0) ++kernelSize;

	cv::Mat result;
	cv::GaussianBlur(_image, result, cv::Size(kernelSize, kernelSize), 0);

	return new OpenCVImage(result);
}

OpenCVImage* OpenCVImage::convertToGrayscale()
{
	if (_image.channels() == 3)
	{
		cv::Mat grayscaleImage;
		cv::cvtColor(_image, grayscaleImage, cv::COLOR_BGR2GRAY);

		return new OpenCVImage(grayscaleImage);
	}

	cv::Mat clonedImage = _image.clone();
	return new OpenCVImage(clonedImage);			// Already in grayscale color
}

OpenCVImage* OpenCVImage::crop(cv::Rect& rectangle)
{
	cv::Mat croppedImage = _image(rectangle).clone();

	return new OpenCVImage(croppedImage);
}

OpenCVImage* OpenCVImage::drawRectangle(Rectangle2D* rectangle)
{
	std::vector<cv::Point> points;
	points.push_back(cv::Point(rectangle->getBottomLeft().x, rectangle->getBottomLeft().y));
	points.push_back(cv::Point(rectangle->getBottomRight().x, rectangle->getBottomRight().y));
	points.push_back(cv::Point(rectangle->getTopRight().x, rectangle->getTopRight().y));
	points.push_back(cv::Point(rectangle->getTopLeft().x, rectangle->getTopLeft().y));

	cv::Mat clonedImage = _image.clone();
	OpenCVImage* result = new OpenCVImage(clonedImage);
	cv::polylines(result->_image, points, true, cv::Scalar(255, 255, 255), 1);

	return result;
}

bool OpenCVImage::enhancedCorrelationCoefficient(OpenCVImage* image, const int motionModel, const unsigned numIterations, const float precisionFactor, cv::Mat& alignmentMatrix)
{
	cv::Mat warpMatrix, resultMat;		// 3x3 or 2x3 depending on the motion model

	if (motionModel == cv::MOTION_HOMOGRAPHY)
	{
		warpMatrix = cv::Mat::eye(3, 3, CV_32F);						// eye => I
	}
	else
	{
		warpMatrix = cv::Mat::eye(2, 3, CV_32F);
	}

	try
	{
		cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, numIterations, precisionFactor);				

		const double ecc = findTransformECC(this->_image, image->_image, warpMatrix, motionModel, criteria);
		alignmentMatrix = warpMatrix.inv();					// Inverse matrix
	}
	catch (cv::Exception& error)
	{
		std::cout << "Could not register images..." << std::endl;

		return false;
	}

	return true;
}

OpenCVImage* OpenCVImage::resize(const ivec2& size)
{
	cv::Size imageSize = cv::Size(size.x, size.y);
	cv::Mat resizedImage;
	cv::resize(_image, resizedImage, imageSize, 0.0, 0.0f, cv::InterpolationFlags::INTER_AREA);

	return new OpenCVImage(resizedImage);
}

bool OpenCVImage::saveImage(const std::string& filename)
{
	if (!this->isEmpty())
	{
		return cv::imwrite(filename, _image);
	}

	return false;
}

OpenCVImage* OpenCVImage::undistort(const mat3& matrixK, const vec3& radialDistortion, const vec2& tangentialDistortion, Rectangle2D& cropRectangle)
{
	OpenCVImage* outcome = nullptr;
	cv::Mat K = cv::Mat::eye(3, 3, CV_32F), distortionCoefficients (1, 5, CV_32F);
	cv::Mat result;

	const vec2 imageSize(_image.cols, _image.rows);
	std::vector<cv::Point2f> pointCorners { cv::Point2f(0.0f, 0.0f), cv::Point2f(imageSize.x - 1.0f, 0.0f), cv::Point2f(0.0f, imageSize.y - 1.0f), cv::Point2f(imageSize.x - 1.0f, imageSize.y - 1.0f)};
	std::vector<cv::Point2f> undistortedPoints;

	for (int row = 0; row < 3; ++row)
	{
		for (int col = 0; col < 3; ++col)
		{
			K.at<float>(cv::Point2i(col, row)) = matrixK[col][row];
		}
	}

	distortionCoefficients.at<float>(cv::Point2i(0, 0)) = radialDistortion[0];
	distortionCoefficients.at<float>(cv::Point2i(1, 0)) = radialDistortion[1];
	distortionCoefficients.at<float>(cv::Point2i(2, 0)) = tangentialDistortion[0];
	distortionCoefficients.at<float>(cv::Point2i(3, 0)) = tangentialDistortion[1];
	distortionCoefficients.at<float>(cv::Point2i(4, 0)) = radialDistortion[2];

	cv::undistort(_image, result, K, distortionCoefficients);
	cv::undistortPoints(pointCorners, undistortedPoints, K, distortionCoefficients, cv::noArray(), K);

	OpenCVImage* undistortedImage = outcome = new OpenCVImage(result);
	vec2 minPoint = vec2(std::max(undistortedPoints[0].x, undistortedPoints[2].x), std::max(undistortedPoints[0].y, undistortedPoints[1].y)),
		 maxPoint = vec2(std::min(undistortedPoints[1].x, undistortedPoints[3].x), std::min(undistortedPoints[2].y, undistortedPoints[3].y));
	bool needCrop = !(minPoint.x <= .0f && minPoint.y <= .0f && maxPoint.x >= imageSize.x && maxPoint.y >= imageSize.y);

	if (needCrop)
	{
		const vec2 rectangleSize = maxPoint - minPoint;
		cv::Rect cvCropRectangle = cv::Rect(minPoint.x, minPoint.y, rectangleSize.x, rectangleSize.y);
		cropRectangle = Rectangle2D(vec2(minPoint), vec2(maxPoint.x, minPoint.y), vec2(minPoint.x, maxPoint.y), vec2(maxPoint));

		outcome = undistortedImage->crop(cvCropRectangle);
		
		delete undistortedImage;
	}

	return outcome;
}

vec3 OpenCVImage::getColor(const ivec2& pixel)
{
	cv::Vec3b color;

	if (_image.channels() == 1)
	{
		color[0] = color[1] = color[2] = _image.at<uchar>(pixel.y, pixel.x);
	}
	else
	{
		color = _image.at<cv::Vec3b>(pixel.y, pixel.x);
	} 

	return vec3(color[2], color[1], color[0]);
}

vec3 OpenCVImage::getColor(const vec2& pixel)
{
	const vec2 imageSize(_image.cols, _image.rows);
	const vec2 minusPixel = glm::clamp(pixel - vec2(0.5f - glm::epsilon<float>()), vec2(.0f), imageSize);
	const vec2 plusPixel = glm::clamp(pixel + vec2(0.5f - glm::epsilon<float>()), vec2(.0f), imageSize);
	const float u = fmod(pixel.x, (int) pixel.x), v = fmod(pixel.y, (int) pixel.y);

	// Access pixels
	const vec3 lowerLeft = this->getColor(ivec2(minusPixel.x, minusPixel.y));
	const vec3 lowerRight = this->getColor(ivec2(plusPixel.x, minusPixel.y));
	const vec3 topLeft = this->getColor(ivec2(minusPixel.x, plusPixel.y));
	const vec3 topRight = this->getColor(ivec2(plusPixel.x, plusPixel.y));

	// Bilinear interpolation
	return (1.0f - u) * (1.0f - v) * lowerLeft + u * (1.0f - v) * lowerRight + (1.0f - u) * v * topLeft + u * v * topRight;
}

float OpenCVImage::getColor(const unsigned row, const unsigned col)
{
	float color;

	color = _image.at<float>(row, col);
	
	return color;
}

/// [Public static methods]

Rectangle2D* OpenCVImage::composeRectangle(Rectangle2D* rectangle, cv::Mat& warpMatrix)
{
	const vec2 rectangleSize = rectangle->getSize();
	std::vector<cv::Point2f> cameraCorners{ cv::Point2f(0.0f, 0.0f), cv::Point2f(rectangleSize.x - 1.0f, 0.0f), 
							          cv::Point2f(0.0f, rectangleSize.y - 1.0f), cv::Point2f(rectangleSize.x - 1.0f, rectangleSize.y - 1.0f) };
	std::vector<cv::Point2f> worldCorners;

	cv::perspectiveTransform(cameraCorners, worldCorners, warpMatrix);

	return new Rectangle2D(vec2(worldCorners[0].x, worldCorners[0].y), vec2(worldCorners[1].x, worldCorners[1].y),
						   vec2(worldCorners[2].x, worldCorners[2].y), vec2(worldCorners[3].x, worldCorners[3].y));
}

/// [Protected methods]