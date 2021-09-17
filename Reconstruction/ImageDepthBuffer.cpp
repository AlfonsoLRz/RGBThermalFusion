#include "stdafx.h"
#include "ImageDepthBuffer.h"

#include "Reconstruction/OpenCVImage.h"

/// [Public methods]

ImageDepthBuffer::ImageDepthBuffer(const ivec2& imageSize, const unsigned numSubdivisions)
	: _cameraPosition(vec3(.0f)), _cellLength(1.0f / numSubdivisions), _depthBufferSize(imageSize * ivec2(numSubdivisions)),
	  _imageSize(imageSize), _maxDepth(FLT_MIN), _minDepth(FLT_MAX)
{
}

ImageDepthBuffer::~ImageDepthBuffer()
{
}

void ImageDepthBuffer::insertPoint(const vec2& point2D, const vec3& point3D, const unsigned pointCloudIndex)
{
	if (!isInside(point2D)) return;

	const ivec2 xy = this->getPosition(point2D);
	const unsigned index = this->getCantorPairing(xy);
	const float depth = glm::distance(_cameraPosition, point3D);

	auto depthIt = _depthBuffer.find(index);

	if (depthIt == _depthBuffer.end() || depth < depthIt->second.first)
	{
		_depthBuffer.insert({ index, std::make_pair(depth, pointCloudIndex) });
	}

#ifdef COMPUTE_MAX_MIN
	_maxDepth = glm::max(depth, _maxDepth);
	_minDepth = glm::min(depth, _minDepth);
#endif
}

void ImageDepthBuffer::resetDepth()
{
	_depthBuffer.clear();
}

void ImageDepthBuffer::saveImage(const std::string& filename)
{
	if (_maxDepth > _minDepth)
	{
		// Collect values and interpolate them between [0, 255]
		std::vector<std::vector<unsigned>> depthBuffer;
		depthBuffer.resize(_depthBufferSize.y);

		for (int row = 0; row < _depthBufferSize.y; ++row)
		{
			depthBuffer[row].resize(_depthBufferSize.x);
			for (int column = 0; column < _depthBufferSize.x; ++column)
			{
				unsigned index = this->getCantorPairing(ivec2(column, row));
				auto depthIt = _depthBuffer.find(index);

				if (depthIt != _depthBuffer.end())
				{
					float depth = depthIt->second.first;
					depth = glm::clamp((_depthBuffer[index].first - _minDepth) / (_maxDepth - _minDepth), .0f, 1.0f);
					depthBuffer[row][column] = std::round(200.0f - depth * 200.0f);
				}
				else
				{
					depthBuffer[row][column] = .0f;
				}
			}
		}

		OpenCVImage* cvImage = new OpenCVImage(depthBuffer);
		cvImage->saveImage(filename);
	}
}

/// [Protected methods]

unsigned ImageDepthBuffer::getCantorPairing(const ivec2& point)
{
	return (1.0f / 2.0f) * (point.x + point.y) * (point.x + point.y + 1) + point.y;
}

ivec2 ImageDepthBuffer::getPosition(const vec2& point)
{
	return ivec2(std::floor(point.x / _cellLength), std::floor(point.y / _cellLength));
}

bool ImageDepthBuffer::isInside(const vec2& point)
{
	return (point.x >= 1 && point.x < _imageSize.x - 1 && point.y >= 1 && point.y < _imageSize.y - 1);
}
