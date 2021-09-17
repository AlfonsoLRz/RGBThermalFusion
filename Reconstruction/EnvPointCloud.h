#pragma once

#include "Geometry/3D/AABB.h"
#include "Graphics/Application/RenderingParameters.h"
#include "Graphics/Core/Model3D.h"
#include "EnvImage.h"

/**
*	@file EnvPointCloud.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 09/24/2020
*/

/**
*	@brief 
*/
class EnvPointCloud : public Model3D
{
public:
	enum PointColorTypes { RGB, RGB_THERMAL, THERMAL, TEMPERATURE, UNUSED, 
						   NUM_COLORS };

protected:
	/**
	*	@return
	*/
	static std::vector<std::string> fromColorTypeToString(PointColorTypes colorType);

protected:
	const static std::string	SECONDARY_BINARY_EXTENSION;			//!<
	const static float			TARGET_SCALE_X;						//!<
	const static std::string	WRITE_POINT_CLOUD_FOLDER;			//!<

protected:
	std::string			_filename;									//!<
	bool				_useBinary;									//!<

	// Spatial information
	AABB				_aabb;										//!<
	vec3				_offset;									//!<
	std::vector<vec4>	_points;									//!<
	std::vector<vec3>	_rgb;										//!<
	std::vector<vec4>	_secondaryColor[NUM_COLORS];				

	// Other properties
	float				_interquartileRange;						//!< 
	float				_meanTemperature;							//!<
	std::vector<GLuint>	_pointClass;								//!<
	float				_q1, _q3;									//!< 
	vec2				_temperatureRange;							//!< 

public:
	/**
	*	@brief Selects the VBO from where the shader will pick color information.
	*/
	static void indicateColorEntry(const bool defaultEntry, RenderingShader* shader, const PointColorTypes colorType = EnvPointCloud::RGB);

	/**
	*	@brief
	*/
	static void indicatePointSize(RenderingShader* shader, RenderingParameters* rendParams, const EnvImage::ImageType imageType = EnvImage::RGB_IMAGE);

protected:
	/**
	*	@brief Computes a triangle mesh buffer composed only by indices.
	*/
	void computeCloudData();

	/**
	*	@brief Fills the content of model component with binary file data.
	*/
	bool loadModelFromBinaryFile();

	/**
	*	@brief Generates geometry via GPU.
	*/
	bool loadModelFromPLY(const mat4& modelMatrix);

	/**
	*	@brief Loads into GPU a new color vector so it can be rendered.
	*/
	bool loadSecondaryColor(const PointColorTypes colorType);

	/**
	*	@brief Loads the PLY point cloud from a binary file, if possible.
	*/
	virtual bool readBinary(const std::string& filename, const std::vector<Model3D::ModelComponent*>& modelComp);

	/**
	*	@brief Communicates the model structure to GPU for rendering purposes.
	*/
	virtual void setVAOData();

	/**
	*	@brief Sorts the temperature array so that quartiles can be computed. 
	*/
	void sortTemperature(std::vector<float>& sortedTemperature);

	/**
	*	@brief
	*/
	void threadedWritePointCloud(const std::string& filename, const bool ascii);

	/**
	*	@brief Writes the model to a binary file in order to fasten the following executions.
	*	@return Success of writing process.
	*/
	virtual bool writeToBinary(const std::string& filename);

public:
	/**
	*	@brief 
	*/
	EnvPointCloud(const std::string& filename, const bool useBinary, const mat4& modelMatrix = mat4(1.0f));

	/**
	*	@brief
	*/
	virtual ~EnvPointCloud();

	/**
	*	@brief  
	*/
	void computeQuartiles();

	/**
	*	@brief Loads the point cloud, either from a binary or a PLY file.
	*	@param modelMatrix Model transformation matrix.
	*	@return True if the point cloud could be properly loaded.
	*/
	virtual bool load(const mat4& modelMatrix = mat4(1.0f));

	/**
	*	@brief Tries to retrieve secondary colors from binary file. 
	*/
	bool loadSecondaryColorsFromBinary();

	/**
	*	@brief Updates the current Axis-Aligned Bounding-Box.
	*/
	void updateBoundaries(const vec3& xyz) { _aabb.update(xyz); }

	/**
	*	@brief
	*/
	float queryTemperature(const vec4& screenColor);

	/**
	*	@brief Writes point cloud as a PLY file.
	*/
	bool writePointCloud(const std::string& filename, const bool ascii);

	/**
	*	@brief Writes secondary colors in a binary file. 
	*/
	bool writeSecondaryColorsToBinary();

	// Getters

	/**
	*	@return
	*/
	AABB getAABB() { return _aabb; }

	/**
	*	@return Path where the point cloud is saved.
	*/
	std::string getFilename() { return _filename; }

	/**
	*	@return
	*/
	std::vector<vec3>* getColors() { return &_rgb; }

	/**
	*	@return
	*/
	std::vector<vec4>* getColors(EnvPointCloud::PointColorTypes colorType) { return &_secondaryColor[colorType]; }

	/**
	*	@return Interquartile range, i.e. Q3 - Q1. 
	*/
	float getInterquartileRange() { return _interquartileRange; }

	/**
	*	@return Mean temperature of the point cloud.
	*/
	float getMeanTemperature() { return _meanTemperature; }

	/**
	*	@brief
	*/
	unsigned getNumberOfPoints() { return unsigned(_points.size()); }

	/**
	*	@brief  
	*/
	std::vector<GLuint>* getPointClass() { return &_pointClass; }

	/**
	*	@return
	*/
	std::vector<vec4>* getPoints() { return &_points; }

	/**
	*	@return Quartiles Q1 and Q3. 
	*/
	vec2 getQ1Q3() { return vec2(_q1, _q3); }

	/**
	*	@brief  
	*/
	vec2 getTemperatureRange() { return _temperatureRange; }

	/// Setters

	/**
	*	@brief Assigns mean temperature (normalized). 
	*/
	void setMean(const float mean) { _meanTemperature = mean; }

	/**
	*	@brief Modifies normals of points.
	*/
	void setNormals(std::vector<vec4>& normals);

	/**
	*	@brief Sets the offset which translates our local system to UTM.
	*/
	void setOffset(const vec3& offset) { _offset = offset; }

	/**
	*	@brief Assigns classes to each point of the 3D structure. 
	*/
	void setPointClass(const std::vector<unsigned>& pointClass);

	/**
	*	@brief Adds secondary colors.
	*/
	void setSecondaryColor(const PointColorTypes colorType, std::vector<vec4>& colors);

	/**
	*	@brief Stores the temperature range. 
	*/
	void setTemperatureRange(const float minTemperature, const float maxTemperature) { _temperatureRange = vec2(minTemperature, maxTemperature); }
};

