#include "stdafx.h"
#include "EnvImage.h"

#include <filesystem>
#include <fstream>
#include <glm/gtx/matrix_transform_2d.hpp>
#include "Reconstruction/EnvCamera.h"
#include "Utilities/FileManagement.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

/// Initialization of static methods

const std::string EnvImage::CSV_ROOT_FOLDER       = "CSV/";
const std::string EnvImage::IMAGES_ROOT_FOLDER    = "Pix4D/Images/";
const std::string EnvImage::RESULT_ECC_FOLDER     = "ECC_Results/";
const std::string EnvImage::ROOT_NAME[]           = { "RGB/", "RGBThermal/", "Thermal/" };

const std::string EnvImage::BINARY_FILES_FOLDER = "Binary/";

std::unordered_map<std::string, EnvImage*>      EnvImage::_globalImage[];
unsigned				                        EnvImage::_globalIdentifier[] = { 0, 0, 0 };
Rectangle2D							            EnvImage::_globalRectangle[];
vec2                                            EnvImage::_maxMinTemperature = vec2(-FLT_MIN, FLT_MAX);

/// Public methods

EnvImage::EnvImage(const std::string& filename, ImageType imageType)
	: _filename(filename), _imageType(imageType), _image(nullptr), _size(0)
{
    for (int image = 0; image < NUM_IMAGE_TYPES; ++image)
    {
        _registered[image] = false;
        _warpMatrix[image] = glm::mat3(1.0f);
    }

    this->registerImage();
}

EnvImage::~EnvImage()
{
    for (int image = 0; image < NUM_IMAGE_TYPES; ++image)
    {
        //
    }

    delete _image;
}

bool EnvImage::isImageAccessible(const EnvImage::ImageType imageType, const vec2& point)
{
    if (imageType == _imageType)
    {
        return _globalRectangle[imageType].isInside(point);
    }
    else
    {
        if (!_registered[imageType])
        {
            return false;
        }

        return _globalRectangle[imageType].isInside(point);
    }

    return true;
}

bool EnvImage::registerImage(const EnvImage::ImageType imageType, const bool saveGraphicResult)
{
    if (imageType == _imageType || !_alignedImage[imageType] || _registered[imageType]) return false;

    // Previously loaded but not accessible
    if (_image->isEmpty() || _alignedImage[imageType]->_image->isEmpty()) return false;
	
    // Registering
    mat3 warpMatrix;
    _registered[imageType] = this->registerImage(_alignedImage[imageType], warpMatrix, saveGraphicResult);
    _warpMatrix[imageType] = warpMatrix;

    if (_registered[imageType])
    {
        std::cout << "Aligned image " << _filename << " ..." << std::endl;
    }

    return true;
}

bool EnvImage::retrieveColor(const EnvImage::ImageType imageType, const vec2& point, vec3& color)
{
    vec3 localPoint = _warpMatrix[imageType] * vec3(point, 1.0f);
    localPoint /= localPoint.z;

    if (!this->isImageAccessible(imageType, localPoint))
    {
        //if (!_registered[imageType])
        //{
        //    return false;
        //}
        return false;
    }

    OpenCVImage* image = (imageType == _imageType) ? _image : _alignedImage[imageType]->_image;
    if (image)
    {
        color = image->getColor(vec2(localPoint));
    }
    else
    {
        return false;
    }

    return true;
}

vec2 EnvImage::transformTo2D(const vec2& point, const EnvImage::ImageType imageType)
{
    return vec2(_warpMatrix[imageType] * vec3(point, 1.0f));
}

// Getters

bool EnvImage::getTemperature(const vec2& pixel, float& temperature)
{
    if (_alignedImage[EnvImage::THERMAL_IMAGE] && _alignedImage[EnvImage::THERMAL_IMAGE]->_temperature.empty()) return .0f;

    vec3 temperatureOffset = vec3(EnvCamera::getTemperatureOffset(), .0f);
    vec3 localPoint = _warpMatrix[EnvImage::THERMAL_IMAGE] * vec3(pixel, 1.0f);
    localPoint /= localPoint.z;
    localPoint += temperatureOffset;

    if (!this->isImageAccessible(EnvImage::THERMAL_IMAGE, localPoint))
    {
        return false;
    }

    temperature = _alignedImage[EnvImage::THERMAL_IMAGE]->getTemperatureValue(ivec2(localPoint));

    return true;
}

void EnvImage::setSize(const ivec2& size)
{
    _size = size;
}

/// Protected methods

std::string EnvImage::getFilenameWithoutExtension()
{
    const size_t dotIndex = _filename.find_last_of('.');

    return _filename.substr(0, dotIndex);
}

unsigned EnvImage::getTemperatureIndex(const ivec2& point)
{
    return point.y * _size.x + point.x;
}

float EnvImage::getTemperatureValue(const vec2& point)
{
    const vec2 imageSize(_size.x, _size.y);
    const vec2 minusPixel = glm::clamp(point - vec2(0.5f - glm::epsilon<float>()), vec2(.0f), imageSize);
    const vec2 plusPixel = glm::clamp(point + vec2(0.5f - glm::epsilon<float>()), vec2(.0f), imageSize);
    const float u = fmod(point.x, (int)point.x), v = fmod(point.y, (int)point.y);

    // Access pixels
    const float lowerLeft = _temperature[this->getTemperatureIndex(ivec2(minusPixel.x, minusPixel.y))];
    const float lowerRight = _temperature[this->getTemperatureIndex(ivec2(plusPixel.x, minusPixel.y))];
    const float topLeft = _temperature[this->getTemperatureIndex(ivec2(minusPixel.x, plusPixel.y))];
    const float topRight = _temperature[this->getTemperatureIndex(ivec2(plusPixel.x, plusPixel.y))];

    // Bilinear interpolation
    return (1.0f - u) * (1.0f - v) * lowerLeft + u * (1.0f - v) * lowerRight + (1.0f - u) * v * topLeft + u * v * topRight;
}

float EnvImage::getTemperatureValue(const ivec2& point)
{
    return _temperature[this->getTemperatureIndex(ivec2(point.x, point.y))];
}

void EnvImage::loadImage(const std::string& projectName, int loadFlags)
{
    if (!_image)
    {
        Rectangle2D cropRectangle;
        std::string filenameWithoutExtension = this->getFilenameWithoutExtension();
        OpenCVImage* startingImage = new OpenCVImage(IMAGES_ROOT_FOLDER + projectName + "/" + ROOT_NAME[_imageType] + filenameWithoutExtension, loadFlags);
    	
        _image = startingImage->undistort(_matrixK, _radialDistortion, _tangentialDistortion, cropRectangle);
        _size = ivec2(_image->getSize());

        delete startingImage;
    }
}

bool EnvImage::readTemperature(const std::string& projectName)
{
    const std::string fileRoot = this->getFilenameWithoutExtension();

    // Read temperature values
    std::string line;
    std::ifstream in;
    ivec2 imageSize = EnvCamera::getOriginalImageSize(EnvImage::THERMAL_IMAGE);
    std::vector<std::string> stringTokens;
    std::vector<float> floatTokens;

    in.open(IMAGES_ROOT_FOLDER + projectName + "/" + CSV_ROOT_FOLDER + ROOT_NAME[_imageType] + fileRoot + CSV_EXTENSION);
    if (!in)
    {
        std::cout << "Could not read temperature values for " << _filename << std::endl;
        return false;
    }

    cv::Mat temperatureMatrix = cv::Mat(imageSize.y, imageSize.x, CV_32F);

    while (!(in >> std::ws).eof())
    {
        std::getline(in, line);

        FileManagement::clearTokens(stringTokens, floatTokens);
        FileManagement::readTokens(line, CSV_DELIMITER, stringTokens, floatTokens);

        if (floatTokens.size())
        {
            temperatureMatrix.at<float>(cv::Point(floatTokens[1], floatTokens[0])) = floatTokens[2];
            this->updateTemperatureBoundaries(floatTokens[2]);
        }
    }

    in.close();

    {
        Rectangle2D rectangle;
        OpenCVImage* image = new OpenCVImage(temperatureMatrix);
        OpenCVImage* undistortedImage = image->undistort(_matrixK, _radialDistortion, _tangentialDistortion, rectangle);
		imageSize = undistortedImage->getSize();
    	
        for (unsigned row = 0; row < imageSize.y; ++row)
        {
            for (unsigned col = 0; col < imageSize.x; ++col)
            {
                _temperature.push_back(undistortedImage->getColor(row, col));
            }
        }

        delete image;
        delete undistortedImage;
    }

    return true;
}

void EnvImage::registerImage()
{
    _id = _globalIdentifier[_imageType]++;
    _globalImage[_imageType][_filename] = this;
}

bool EnvImage::registerImage(EnvImage* image, mat3& glmWarpMatrix, bool saveResult)
{
    bool success = false;

    const float rgbGSD = EnvCamera::getGSD(EnvImage::RGB_THERMAL_IMAGE, 1.0f);
    const float thermalGSD = EnvCamera::getGSD(EnvImage::THERMAL_IMAGE, 1.0f);
    const vec2 shrinkK = EnvCamera::getImageCropScale(EnvImage::RGB_THERMAL_IMAGE);
	const vec2 offset = EnvCamera::getImageCropOffset(EnvImage::RGB_THERMAL_IMAGE);
    vec2 rgbSize = this->getSize(), thermalSize = image->getSize();
    vec2 center = rgbSize / 2.0f + offset;
	
    vec2 newSize = vec2(rgbSize.x * shrinkK.x, rgbSize.y * shrinkK.y);
    vec2 newSize_2 = newSize / 2.0f;
    vec2 shrinkSize = thermalSize;
    vec2 raspect = newSize / shrinkSize;
    cv::Rect cropRectangle = cv::Rect(center.x - newSize_2.x, center.y - newSize_2.y, newSize.x, newSize.y);
    cv::Mat warpMatrix;

    OpenCVImage* grayscaleRGBImage = this->_image->convertToGrayscale();
    OpenCVImage* grayscaleThermalImage = image->_image->convertToGrayscale();

    OpenCVImage* croppedImage = grayscaleRGBImage->crop(cropRectangle);
    OpenCVImage* shrinkedImage_rgb = croppedImage->resize(shrinkSize);
    OpenCVImage* shrinkedImage_thermal = grayscaleThermalImage->resize(shrinkSize);

    bool alignedThermal = shrinkedImage_rgb->enhancedCorrelationCoefficient(shrinkedImage_thermal, cv::MOTION_HOMOGRAPHY, NUM_ECC_ITERATIONS, ECC_PRECISION, warpMatrix);

    // Compose rectangle and apply transformations if images were aligned
    if (alignedThermal)
    {
        Rectangle2D* area = new Rectangle2D(vec2(.0f), vec2(shrinkSize.x, .0f), vec2(.0f, shrinkSize.y), shrinkSize);         // Initial area
        Rectangle2D* rotatedArea = OpenCVImage::composeRectangle(area, warpMatrix);
        rotatedArea->translateCorners(-shrinkSize / 2.0f);
        rotatedArea->scaleCorners(raspect);
        rotatedArea->translateCorners(center);              // From (0, 0) to center

        if (!rotatedArea->exceedsSlope(RECTANGLE_THRESHOLD.x, RECTANGLE_THRESHOLD.y))
        {
            if (saveResult)
            {
                OpenCVImage* result = grayscaleRGBImage->drawRectangle(rotatedArea);
                result->saveImage(RESULT_ECC_FOLDER + _filename);

                delete result;
            }

            // Build matrix to get the inverse process
            {
                cv::Mat invMatrix = warpMatrix.inv();
                mat3 glmInvMatrix = mat3(1.0f);

                // Fill warp matrix of imageType with the result: glm has columnwise access, while OpenCV access first to the rows
                for (int row = 0; row < 3; ++row)
                {
                    for (int col = 0; col < 3; ++col)
                    {
                        glmInvMatrix[row][col] = invMatrix.at<float>(cv::Point(row, col));
                    }
                }

                glmWarpMatrix = glmInvMatrix * glm::translate(mat3(1.0f), shrinkSize / 2.0f) * glm::scale(mat3(1.0f), 1.0f / raspect) * glm::translate(mat3(1.0f), -center);
            }

            success = true;
        }

        delete area;
        delete rotatedArea;
    }

    // Delete pointers to images
    delete grayscaleRGBImage;
    delete grayscaleThermalImage;
    delete croppedImage;
    delete shrinkedImage_rgb;
    delete shrinkedImage_thermal;

    return success;
}

vec2 EnvImage::transformPoint(EnvImage::ImageType imageType, const vec2& point)
{
    return _warpMatrix[imageType] * vec3(point, 1.0f);
}

/// [Static protected methods]

std::string EnvImage::getPairString(const EnvImage::ImageType image1, const EnvImage::ImageType image2)
{
    const unsigned min = std::min(image1, image2), max = std::max(image1, image2);

    return imageToString(image1) + imageToString(image2);
}

/// [Static public methods]

void EnvImage::buildRectangles()
{
    for (int imageIdx = 0; imageIdx < NUM_IMAGE_TYPES; ++imageIdx)
    {
        auto cameras = EnvCamera::getCameras(static_cast<EnvImage::ImageType>(imageIdx));

    	if (!cameras->empty())
    	{
            vec2 imageSize = cameras->at(0)->getImage()->getSize();
            _globalRectangle[imageIdx] = Rectangle2D(vec2(RECTANGLE_OFFSET), vec2(imageSize.x - RECTANGLE_OFFSET, RECTANGLE_OFFSET),
													 vec2(RECTANGLE_OFFSET, imageSize.y - RECTANGLE_OFFSET), imageSize - RECTANGLE_OFFSET);
    	}
    }
}

bool EnvImage::loadAlignmentMatrices(const std::string& projectName, const EnvImage::ImageType imageIdx_01, const EnvImage::ImageType imageIdx_02)
{
    if (imageIdx_01 == imageIdx_02) return false;

    const std::string filename = IMAGES_ROOT_FOLDER + projectName + "/" + BINARY_FILES_FOLDER + EnvImage::getPairString(static_cast<EnvImage::ImageType>(imageIdx_01),
                                                                                    static_cast<EnvImage::ImageType>(imageIdx_02)) + BINARY_EXTENSION;

    std::ifstream fin(filename, std::ios::in | std::ios::binary);

    if (fin.is_open())
    {
        std::vector<ImageNameChar> imageNames;
        std::vector<mat3> matrices;
        std::vector<bool> registered;
        size_t numRectangles;

        fin.read((char*)&numRectangles, sizeof(size_t));

        imageNames.resize(numRectangles);
        fin.read((char*)&imageNames[0], numRectangles * sizeof(ImageNameChar));

        matrices.resize(numRectangles);
        fin.read((char*)&matrices[0], numRectangles * sizeof(mat3));

        fin.close();

        for (int k = 0; k < numRectangles; ++k)
        {
            std::string imageName = imageNames[k].toString();
            auto imageIt = _globalImage[imageIdx_01].find(imageName);

            if (imageIt != _globalImage[imageIdx_01].end())
            {
                imageIt->second->_warpMatrix[imageIdx_02] = matrices[k];
                imageIt->second->_registered[imageIdx_02] = true;
            }
        }

        return true;
    }

    return false;
}

bool EnvImage::loadBinaryTemperatureData(const std::string& projectName)
{
    const std::string filename = IMAGES_ROOT_FOLDER + projectName + "/" + BINARY_FILES_FOLDER + EnvImage::imageToString(static_cast<EnvImage::ImageType>(EnvImage::THERMAL_IMAGE))
                                 + TEMPERATURE_EXTENSION + BINARY_EXTENSION;

    std::ifstream fin(filename, std::ios::in | std::ios::binary);

    if (fin.is_open())
    {
        size_t numImages;
        ivec2 size;
        std::vector<ImageNameChar> imageNames;
        std::vector<std::vector<float>> temperatureImage;

        fin.read((char*)&numImages, sizeof(size_t));
        fin.read((char*)&size, sizeof(ivec2));
        fin.read((char*)&_maxMinTemperature, sizeof(vec2));

        imageNames.resize(numImages);
        fin.read((char*)&imageNames[0], numImages * sizeof(ImageNameChar));

        temperatureImage.resize(numImages);
        for (int k = 0; k < numImages; ++k)
        {
            temperatureImage[k].resize(size.x * size.y);
            fin.read((char*)&temperatureImage[k][0], sizeof(float) * size.x * size.y);
        }

        for (int k = 0; k < numImages; ++k)
        {
            std::string imageName = imageNames[k].toString();
            auto imageIt = _globalImage[EnvImage::THERMAL_IMAGE].find(imageName);

            if (imageIt != _globalImage[EnvImage::THERMAL_IMAGE].end())
            {
                imageIt->second->_temperature = std::move(temperatureImage[k]);
            }
        }

        fin.close();

        return true;
    }

    return false;
}

void EnvImage::loadImages(const std::string& projectName, const EnvImage::ImageType imageType, int loadFlags)
{
    auto imageIt = _globalImage[imageType].begin();

    while (imageIt != _globalImage[imageType].end())
    {
        imageIt->second->loadImage(projectName, loadFlags);
        ++imageIt;
    }
}

void EnvImage::loadTemperatureData(const std::string& projectName)
{
    auto imageIt = _globalImage[EnvImage::THERMAL_IMAGE].begin();

    while (imageIt != _globalImage[EnvImage::THERMAL_IMAGE].end())
    {
        imageIt->second->readTemperature(projectName);

        std::cout << "Read temperature from image " << imageIt->second->getName() << std::endl;

        ++imageIt;
    }
}

bool EnvImage::saveAlignmentRectangle(const std::string& projectName, const EnvImage::ImageType imageIdx_01, const EnvImage::ImageType imageIdx_02, bool overwrite)
{
    if (imageIdx_01 == imageIdx_02) return false;

    const std::string filename = IMAGES_ROOT_FOLDER + projectName + "/" + BINARY_FILES_FOLDER + EnvImage::getPairString(static_cast<EnvImage::ImageType>(imageIdx_01),
                                                                                    static_cast<EnvImage::ImageType>(imageIdx_02)) + BINARY_EXTENSION;

    if (std::filesystem::exists(filename) && !overwrite) return false;        // Already saved binary file

    // Gather all the rectangles
    std::vector<ImageNameChar> imageNames;
    std::vector<mat3> matrices;

    auto imageIt = _globalImage[imageIdx_01].begin();

    while (imageIt != _globalImage[imageIdx_01].end())
    {
        if (imageIt->second->_registered[imageIdx_02])
        {
            ImageNameChar imageName;
            imageName.fromString(imageIt->second->_filename);
            imageNames.push_back(imageName);
            matrices.push_back(imageIt->second->_warpMatrix[imageIdx_02]);
        }

        ++imageIt;
    }

    if (imageNames.size())
    {
        std::ofstream fout(filename, std::ios::out | std::ios::binary);

        if (fout.is_open())
        {
            const size_t numRectangles = imageNames.size();

            fout.write((char*)&numRectangles, sizeof(size_t));
            fout.write((char*)&imageNames[0], numRectangles * sizeof(ImageNameChar));
            fout.write((char*)&matrices[0], numRectangles * sizeof(mat3));            // 3x3

            fout.close();

            return true;
        }
    }

    return false;
}

bool EnvImage::saveTemperatureData(const std::string& projectName, bool overwrite)
{
    const std::string filename = IMAGES_ROOT_FOLDER + projectName + "/" + BINARY_FILES_FOLDER + EnvImage::imageToString(static_cast<EnvImage::ImageType>(EnvImage::THERMAL_IMAGE))
                                 + TEMPERATURE_EXTENSION + BINARY_EXTENSION;

    if (std::filesystem::exists(filename) && !overwrite) return false;        // Already saved binary file

    // Gather all the rectangles
    std::vector<ImageNameChar> imageNames;
    std::vector<std::vector<float>> temperatureImage;
    ImageNameChar imageName;

    auto imageIt = _globalImage[EnvImage::THERMAL_IMAGE].begin();

    while (imageIt != _globalImage[EnvImage::THERMAL_IMAGE].end())
    {
        if (imageIt->second->_temperature.size())
        {
            imageName.fromString(imageIt->second->_filename);
            imageNames.push_back(imageName);
            temperatureImage.push_back(imageIt->second->_temperature);
        }

        ++imageIt;
    }

    if (imageNames.size())
    {
        std::ofstream fout(filename, std::ios::out | std::ios::binary);

        if (fout.is_open())
        {
            const size_t numImages = imageNames.size();
            const ivec2 imageSize = _globalImage[EnvImage::THERMAL_IMAGE].begin()->second->getSize();

            fout.write((char*)&numImages, sizeof(size_t));
            fout.write((char*)&imageSize, sizeof(ivec2));
            fout.write((char*)&_maxMinTemperature, sizeof(vec2));
            fout.write((char*)&imageNames[0], numImages * sizeof(ImageNameChar));

            for (int i = 0; i < temperatureImage.size(); ++i)
            {
                const size_t numTemperatureValues = temperatureImage[i].size();
                fout.write((char*)&temperatureImage[i][0], sizeof(float) * numTemperatureValues);
            }

            fout.close();

            return true;
        }
    }

    return false;
}

void EnvImage::updateTemperatureBoundaries(const float temperature)
{
    _maxMinTemperature = vec2(glm::max(_maxMinTemperature.x, temperature), glm::min(_maxMinTemperature.y, temperature));
}
