#pragma once

/**
*	@file ImageDepthBuffer.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 27/01/2021
*/

#define COMPUTE_MAX_MIN false
#define DEPTH_OFFSET 10.0f

typedef std::unordered_map<unsigned, std::pair<float, unsigned>> DepthBuffer;

/**
*	@brief
*/
class ImageDepthBuffer
{
protected:
	DepthBuffer	_depthBuffer;				//!<

	vec3		_cameraPosition;			//!<
	float		_cellLength;				//!<
	ivec2		_depthBufferSize;			//!<
	ivec2		_imageSize;					//!<

	// Preprocessor dependant
	float		_maxDepth, _minDepth;		//!<

protected:
	/**
	*	@return
	*/
	unsigned getCantorPairing(const ivec2& point);

	/**
	*	@return  
	*/
	ivec2 getPosition(const vec2& point);

	/**
	*	@return 
	*/
	bool isInside(const vec2& point);

public:
	/**
	*	@brief Constructor.
	*/
	ImageDepthBuffer(const ivec2& imageSize, const unsigned numSubdivisions);

	/**
	*	@brief Destructor.
	*/
	~ImageDepthBuffer();

	/**
	*	@brief 
	*/
	void insertPoint(const vec2& point2D, const vec3& point3D, const unsigned pointCloudIndex);

	/**
	*	@brief 
	*/
	void resetDepth();

	/**
	*	@brief 
	*/
	void saveImage(const std::string& filename);

	// Getters

	/**
	*	@return 
	*/
	DepthBuffer* getVisiblePoints() { return &_depthBuffer; }

	// Setters

	/**
	*	@brief 
	*/
	void setCameraPosition(const vec3& cameraPosition) { _cameraPosition = cameraPosition; }
};

