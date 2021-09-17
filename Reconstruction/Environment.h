#pragma once

#include "EnvCamera.h"
#include "EnvPointCloud.h"
#include "EnvImage.h"

#include "DataStructures/OctreePointCloud.h"
#include "DataStructures/PointCloudBVH.h"
#include "DataStructures/RadiusOctree.h"
#include "Graphics/Core/CADModel.h"
#include "Graphics/Core/DrawBox3D.h"
#include "Graphics/Core/DrawOctreePointCloud.h"
#include "Graphics/Core/DrawRadiusOctree.h"
#include "Reconstruction/ImageDepthBuffer.h"
#include "Reconstruction/PointCloudClassifier.h"
#include "Reconstruction/ThermalAugmentation.h"

/**
*	@file Environment.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 09/25/2020
*/

#define DELIMITER ' '

#define BOX_CENTER_HEADER "#Center"
#define BOX_SIZE_HEADER "#Size"

#define CAMERA_TRANSLATION_HEADER "camera position t [3x1]"
#define CAMERA_ROTATION_HEADER "camera rotation R [3x3]"

#define CAMERA_FOCAL_LENGTH "FOCAL"
#define CAMERA_OFFSET_MM "#Principal Point Offset xpoff ypoff in mm"
#define CAMERA_OFFSET_PIXEL "#Principal Point Offset xpoff ypoff in pixel"
#define CAMERA_SENSOR_WIDTH "sensor width"

#define CAMERA_SYMMETRICAL_DISTORTION "#Symmetrical Lens Distortion Coeffs"
#define CAMERA_TANGENTIAL_DISTORTION "#Tangential Lens Distortion Coeffs"

#define IMAGE_CROP_OFFSET "#Crop Offset"
#define IMAGE_CROP_SCALE "#Crop Scale"

#define IMAGE_OFFSET "#Offset"

#define CHECK_VISIBILITY		false
#define CHECK_OCCLUSION			false
#define AUGMENTATE_POINT_CLOUD	true

typedef std::map<std::string, EnvCamera*> CameraMap;

/**
*	@brief
*/
class Environment
{
protected:
	enum ThermalAugmentationType: uint8_t
	{
		NAIVE = 0, DEPTH_BUFFER_VISIBILITY = 1, OCCLUSION = 2
	};
protected:
	// Main folders
	const static std::string SCENE_ROOT_FOLDER;									//!<

	static const std::string IMAGE_TYPE_FOLDER[NUM_IMAGE_TYPES];				//!<

	// Root name for each type of image and their files
	const static std::string ROOT_NAME[NUM_IMAGE_TYPES];						//!<

	// Pix4D-computed parameters
	static const std::string CAMERA_FOLDER;										//!< 										
	static const std::string CALIBRATED_CAMERA_PARAMETERS;						//!<
	static const std::string CALIBRATED_IMAGE_POSITION;							//!<
	static const std::string CALIBRATED_EXTERNAL_PARAMETERS;					//!<
	static const std::string CALIBRATED_INTERNAL_PARAMETERS;					//!<
	static const std::string CALIBRATED_PIX4D_INTERNAL_PARAMETERS;				//!<
	static const std::string IMAGE_CROP_PARAMS;									//!<
	static const std::string OFFSET;											//!<
	static const std::string PMATRIX;											//!<
	static const std::string TEMPERATURE_OFFSET;								//!<

	// Point cloud
	const static std::string POINT_CLOUD_FOLDER;								//!<
	const static std::string POINT_CLOUD_FILE[NUM_IMAGE_TYPES];					//!<

	// Study area boxes
	const static std::string STUDY_AREA_FOLDER;									//!<

	// Applicators
	const static std::vector<std::unique_ptr<ThermalAugmentation>> THERMAL_AUGMENTATION_APPLICATOR;		//!<

protected:
	std::string				_projectName;										//!<

	// Read attributes
	CameraMap				_camera[NUM_IMAGE_TYPES];							//!<
	vec3					_offset[NUM_IMAGE_TYPES];							//!<

	// Scenario
	EnvPointCloud*			_pointCloud[NUM_IMAGE_TYPES];						//!<
	OctreePointCloud*		_octree[NUM_IMAGE_TYPES];							//!<
	DrawOctreePointCloud*	_drawOctree[NUM_IMAGE_TYPES];						//!<

	// Point cloud
	ThermalAugmentationType _thermalAugmentationType;							//!< 

	// Point cloud data structures
	PointCloudBVH*			_pointCloudBVH[NUM_IMAGE_TYPES];					//!< 
	RadiusOctree*			_radiusOctree[NUM_IMAGE_TYPES];						//!<
	DrawRadiusOctree*		_drawRadiusOctree[NUM_IMAGE_TYPES];					//!<
	
	// Point cloud decomposition
	PointCloudClassifier*	_classifier[NUM_IMAGE_TYPES];						//!<

protected:
	/**
	*	@return Camera identified by imageName. If it does not exist, this function creates it.
	*/
	EnvCamera* getCamera(const std::string& imageName, const EnvImage::ImageType imageType);

	/**
	*	@return Applicators for thermal augmentation methods.
	*/
	static std::vector<std::unique_ptr<ThermalAugmentation>> getThermalAugmentationApplicators();

protected:
	// Specific reading functions

	/**
	*	@brief Reads the calibrated parameters for each image.
	*/
	void readCameraCalibratedParameters(const std::string& filename, const EnvImage::ImageType imageType);

	/**
	*	@brief Reads the calibrated position for each image.
	*/
	void readCameraCalibratedImagesPosition(const std::string& filename, const EnvImage::ImageType imageType);

	/**
	*	@brief Reads the external calibrated parameters for each image.
	*/
	void readCameraExternalCalibratedParameters(const std::string& filename, const EnvImage::ImageType imageType);

	/**
	*	@brief Reads the internal calibrated parameters for each image.
	*/
	void readCameraInternalCalibratedParameters(const std::string& filename, const EnvImage::ImageType imageType);

	/**
	*	@brief Reads offset for images so they can be registered faster. 
	*/
	void readImageCropArea(const std::string& filename, const EnvImage::ImageType imageType);

	/**
	*	@brief
	*/
	void readMetadata(EnvImage::ImageType imageType);

	/**
	*	@brief Reads UTM offset for the whole project.
	*/
	void readOffset(const std::string& filename, const EnvImage::ImageType imageType);

	/**
	*	@brief Reads from 'filename' file cameras' matrices to transform 3D points into 2D coordinates.
	*/
	void readPMatrices(const std::string& filename, const EnvImage::ImageType imageType);

	/**
	*	@brief Reads both RGB and thermal point clouds.
	*/
	void readPointClouds(const EnvImage::ImageType imageType);

	/**
	*	@brief Reads translation between thermal and temperature data. 
	*/
	void readTemperatureOffset(const std::string& filename);

protected:
	/**
	*	@brief Classifies read point clouds so that we can distinguish several classes.
	*/
	void classifyPointClouds();

	/**
	*	@brief Register images once them all are loaded.
	*/
	void registerImages();

public:
	/**
	*	@brief Main constructor.
	*/
	Environment(const std::string& projectName);

	/**
	*	@brief Destructor.
	*/
	virtual ~Environment();

	/**
	*	@brief Computes temperature for a RGB point cloud. 
	*/
	ThermalAugmentation::VolatileAugmentationData* augmentate3DPoints();

	/**
	*	@brief Loads point cloud and image data.
	*/
	void load();

	// Getters

	/**
	*	@return RGB point cloud imported from a PLY file.
	*/
	EnvPointCloud* getPointCloud(const EnvImage::ImageType imageType) { return _pointCloud[imageType]; }

	/**
	*	@return 
	*/
	DrawOctreePointCloud* getPointCloudOctree(const EnvImage::ImageType imageType) { return _drawOctree[imageType]; }

	/**
	*	@return
	*/
	DrawRadiusOctree* getPointCloudRadiusOctree(const EnvImage::ImageType imageType) { return _drawRadiusOctree[imageType]; }
};

