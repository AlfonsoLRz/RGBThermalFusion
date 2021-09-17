#include "stdafx.h"
#include "PointCloudKNN.h"

#include "Geometry/General/CGAL.h"
#include "Graphics/Core/ComputeShader.h"
#include "Graphics/Core/ShaderList.h"
#include "Reconstruction/EnvCamera.h"

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>

#include "Utilities/ChronoUtilities.h"

// [Static variables]

const unsigned PointCloudKNN::K_NEIGHBORS = 10;
const unsigned PointCloudKNN::MAX_POINT_CLOUD_SIZE = 6e6;
const unsigned PointCloudKNN::SEARCH_RADIUS = 3000;

// [Public methods]

PointCloudKNN::PointCloudKNN(std::vector<vec4>* points, const AABB& aabb, const unsigned kNeighbors, const unsigned searchRadius) :
	_aabb(aabb), _points(points), _gpuData(nullptr), _numNeighbors(kNeighbors), _searchRadius(searchRadius)
{
	_maxDistance = 0.15f;
	_viewPoint = _aabb.center() * vec3(.0f, .0f, 1.0f) + _aabb.extent() * vec3(.0f, .0f, 10.0f);
}

PointCloudKNN::~PointCloudKNN()
{
}

void PointCloudKNN::computeNormals(std::vector<vec4>& normals)
{
	ChronoUtilities::initChrono();
	
#if ESTIMATE_NORMAL_GPU
	this->computeNormalsGPU(normals);
#else
	this->computeNormalsPCL(normals);
#endif

	std::cout << ChronoUtilities::getDuration() << std::endl;
}

// [Protected methods]

void PointCloudKNN::aggregateSSBOData()
{
	_gpuData->_pointCloudSSBO = ComputeShader::setReadBuffer(*_points, GL_STATIC_DRAW);
	_gpuData->_mortonCodesSSBO = this->calculateMortonCodes();
	_gpuData->_sortedIndicesSSBO = this->sortFacesByMortonCode();
}

GLuint PointCloudKNN::calculateMortonCodes()
{
	ComputeShader* computeMortonShader = ShaderList::getInstance()->getComputeShader(RendEnum::COMPUTE_MORTON_CODES_PCL);

	const unsigned arraySize = _points->size();
	const int numGroups = ComputeShader::getNumGroups(arraySize);
	const GLuint mortonCodeBuffer = ComputeShader::setWriteBuffer(unsigned(), arraySize);

	computeMortonShader->bindBuffers(std::vector<GLuint> { _gpuData->_pointCloudSSBO, mortonCodeBuffer });
	computeMortonShader->use();
	computeMortonShader->setUniform("arraySize", arraySize);
	computeMortonShader->setUniform("sceneMaxBoundary", _aabb.max());
	computeMortonShader->setUniform("sceneMinBoundary", _aabb.min());
	computeMortonShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	return mortonCodeBuffer;
}

void PointCloudKNN::computeNormalsCrossEstimation(std::vector<vec4>& normals, const unsigned offset)
{
	ComputeShader* computeNormalShader = ShaderList::getInstance()->getComputeShader(RendEnum::COMPUTE_PC_NORMAL);

	const unsigned arraySize = _gpuData->_arraySize;
	const int numGroups = ComputeShader::getNumGroups(arraySize);
	const int maxGroupSize = ComputeShader::getMaxGroupSize();
	_gpuData->_normalSSBO = ComputeShader::setWriteBuffer(vec4(), arraySize, GL_DYNAMIC_DRAW);

	computeNormalShader->bindBuffers(std::vector<GLuint> { _gpuData->_pointCloudSSBO, _gpuData->_neighborsSSBO, _gpuData->_normalSSBO });
	computeNormalShader->use();
	computeNormalShader->setUniform("arraySize", arraySize);
	computeNormalShader->setUniform("numNeighbors", _numNeighbors);
	computeNormalShader->setUniform("offset", offset);
	computeNormalShader->setUniform("viewpoint", _viewPoint);
	computeNormalShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

	vec4* normalPointer = ComputeShader::readData(_gpuData->_normalSSBO, vec4());
	normals.insert(normals.end(), normalPointer, normalPointer + arraySize);
}

void PointCloudKNN::computeNormalsCGAL(std::vector<vec4>& normals)
{
	std::list<PointVectorPairCGAL> points;

	for (vec4& point : *_points)
	{
		points.push_back(std::make_pair(PointCGAL(point.x, point.y, point.z), VectorCGAL(.0f, .0f, .0f)));
	}

	// Estimate normals with a fixed radius
	CGAL::pca_estimate_normals<ConcurrencyTag>
				(points,
				K_NEIGHBORS,
				CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPairCGAL>()).
				normal_map(CGAL::Second_of_pair_property_map<PointVectorPairCGAL>()));			

	// Orients normals. mst_orient_normals() requires a range of points as well as property maps to access each point's position and normal
	std::list<PointVectorPairCGAL>::iterator unoriented_points_begin = CGAL::mst_orient_normals(points, K_NEIGHBORS, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPairCGAL>()).normal_map(CGAL::Second_of_pair_property_map<PointVectorPairCGAL>()));

	for (PointVectorPairCGAL& pointNormal: points)
	{
		normals.push_back(vec4(glm::normalize(vec3(pointNormal.second.hx(), pointNormal.second.hy(), pointNormal.second.hz())), .0f));
	}
}

void PointCloudKNN::computeNormalsGPU(std::vector<vec4>& normals)
{
	_gpuData = new VolatileGPUData();
	
	const unsigned		numPoints = _points->size();
	const unsigned		minSize = std::min(MAX_POINT_CLOUD_SIZE, numPoints);
	const unsigned		numGroups = ComputeShader::getNumGroups(minSize);
	unsigned			leftPoints = numPoints;

	this->aggregateSSBOData();

	while (leftPoints > 0)
	{
		_gpuData->_arraySize = std::min(MAX_POINT_CLOUD_SIZE, leftPoints);

		this->searchKNeighbors(numPoints - leftPoints);
#if USE_SVD
		this->computeNormalsSVD(normals);
#else
		this->computeNormalsCrossEstimation(normals, numPoints - leftPoints);
#endif
		
		leftPoints -= _gpuData->_arraySize;
	}

	delete _gpuData;
}

void PointCloudKNN::computeNormalsPCL(std::vector<vec4>& normals)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (vec4& point: *_points)
	{
		pcl::PointXYZ pclPoint(point.x, point.y, point.z);
		cloud->push_back(pclPoint);
	}

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// Create an empty kdtree representation
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(_maxDistance);
	ne.setViewPoint(_viewPoint.x, _viewPoint.y, _viewPoint.z);
	ne.compute(*cloud_normals);

	for (pcl::Normal& normal: *cloud_normals)
	{
		normals.push_back(vec4(glm::normalize(vec3(normal.normal_x, normal.normal_y, normal.normal_z)), .0f));
	}
}

void PointCloudKNN::computeNormalsSVD(std::vector<vec4>& normals)
{
	ComputeShader* computeNormalShader = ShaderList::getInstance()->getComputeShader(RendEnum::SVD_ESTIMATION);

	const unsigned arraySize = _gpuData->_arraySize;
	const int numGroups = ComputeShader::getNumGroups(arraySize);
	const int maxGroupSize = ComputeShader::getMaxGroupSize();
	_gpuData->_normalSSBO = ComputeShader::setWriteBuffer(vec4(), arraySize, GL_DYNAMIC_DRAW);

	computeNormalShader->bindBuffers(std::vector<GLuint> { _gpuData->_pointCloudSSBO, _gpuData->_neighborsSSBO, _gpuData->_normalSSBO });
	computeNormalShader->use();
	computeNormalShader->setUniform("arraySize", arraySize);
	computeNormalShader->setUniform("numNeighbors", _numNeighbors);
	computeNormalShader->setUniform("viewpoint", _aabb.center() * vec3(.0f, .0f, 1.0f) + _aabb.extent() * vec3(.0f, .0f, 10.0f));
	computeNormalShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

	vec4* normalPointer = ComputeShader::readData(_gpuData->_normalSSBO, vec4());
	normals.insert(normals.end(), normalPointer, normalPointer + arraySize);
}

void PointCloudKNN::searchKNeighbors(const unsigned offset)
{
	ComputeShader* KNNShader	= ShaderList::getInstance()->getComputeShader(RendEnum::FIND_BEST_K_NEIGHBORS);

	const unsigned arraySize	= _gpuData->_arraySize;
	const int numGroups			= ComputeShader::getNumGroups(arraySize);
	const int maxGroupSize		= ComputeShader::getMaxGroupSize();
	_gpuData->_neighborsSSBO	= ComputeShader::setWriteBuffer(GLuint(), arraySize * _numNeighbors, GL_DYNAMIC_DRAW);

	KNNShader->bindBuffers(std::vector<GLuint> { _gpuData->_pointCloudSSBO, _gpuData->_sortedIndicesSSBO, _gpuData->_neighborsSSBO });
	KNNShader->use();
	KNNShader->setUniform("arraySize", arraySize);
	KNNShader->setUniform("maxWorldDistance", _maxDistance);
	KNNShader->setUniform("numNeighbors", _numNeighbors);
	KNNShader->setUniform("numPoints", unsigned(_points->size()));
	KNNShader->setUniform("offset", offset);
	KNNShader->setUniform("radius", _searchRadius);
	KNNShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);
}

GLuint PointCloudKNN::sortFacesByMortonCode()
{
	ComputeShader* bitMaskShader = ShaderList::getInstance()->getComputeShader(RendEnum::BIT_MASK_RADIX_SORT);
	ComputeShader* reduceShader = ShaderList::getInstance()->getComputeShader(RendEnum::REDUCE_PREFIX_SCAN);
	ComputeShader* downSweepShader = ShaderList::getInstance()->getComputeShader(RendEnum::DOWN_SWEEP_PREFIX_SCAN);
	ComputeShader* resetPositionShader = ShaderList::getInstance()->getComputeShader(RendEnum::RESET_LAST_POSITION_PREFIX_SCAN);
	ComputeShader* reallocatePositionShader = ShaderList::getInstance()->getComputeShader(RendEnum::REALLOCATE_RADIX_SORT);

	const unsigned numBits = 30;			// 10 bits per coordinate (3D)
	unsigned arraySize = _points->size();
	unsigned currentBits = 0;
	const int numGroups = ComputeShader::getNumGroups(arraySize);
	const int maxGroupSize = ComputeShader::getMaxGroupSize();
	GLuint* indices = new GLuint[arraySize];

	// Binary tree parameters
	const unsigned startThreads = unsigned(std::ceil(arraySize / 2.0f));
	const unsigned numExec = unsigned(std::ceil(std::log2(arraySize)));
	const unsigned numGroups2Log = unsigned(ComputeShader::getNumGroups(startThreads));
	unsigned numThreads = 0, iteration;

	// Fill indices array from zero to arraySize - 1
	for (int i = 0; i < arraySize; ++i) { indices[i] = i; }

	GLuint indicesBufferID_1, indicesBufferID_2, pBitsBufferID, nBitsBufferID, positionBufferID;
	indicesBufferID_1 = ComputeShader::setWriteBuffer(GLuint(), arraySize);
	indicesBufferID_2 = ComputeShader::setReadBuffer(indices, arraySize);					// Substitutes indicesBufferID_1 for the next iteration
	pBitsBufferID = ComputeShader::setWriteBuffer(GLuint(), arraySize);
	nBitsBufferID = ComputeShader::setWriteBuffer(GLuint(), arraySize);
	positionBufferID = ComputeShader::setWriteBuffer(GLuint(), arraySize);

	while (currentBits < numBits)
	{
		std::vector<GLuint> threadCount{ startThreads };
		threadCount.reserve(numExec);

		std::swap(indicesBufferID_1, indicesBufferID_2);							// indicesBufferID_2 is initialized with indices cause it's swapped here

		// FIRST STEP: BIT MASK, check if a morton code gives zero or one for a certain mask (iteration)
		unsigned bitMask = 1 << currentBits++;

		bitMaskShader->bindBuffers(std::vector<GLuint> { _gpuData->_mortonCodesSSBO, indicesBufferID_1, pBitsBufferID, nBitsBufferID });
		bitMaskShader->use();
		bitMaskShader->setUniform("arraySize", arraySize);
		bitMaskShader->setUniform("bitMask", bitMask);
		bitMaskShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

		// SECOND STEP: build a binary tree with a summatory of the array
		reduceShader->bindBuffers(std::vector<GLuint> { nBitsBufferID });
		reduceShader->use();
		reduceShader->setUniform("arraySize", arraySize);

		iteration = 0;
		while (iteration < numExec)
		{
			numThreads = threadCount[threadCount.size() - 1];

			reduceShader->setUniform("iteration", iteration++);
			reduceShader->setUniform("numThreads", numThreads);
			reduceShader->execute(numGroups2Log, 1, 1, maxGroupSize, 1, 1);

			threadCount.push_back(std::ceil(numThreads / 2.0f));
		}

		// THIRD STEP: set last position to zero, its faster to do it in GPU than retrieve the array in CPU, modify and write it again to GPU
		resetPositionShader->bindBuffers(std::vector<GLuint> { nBitsBufferID });
		resetPositionShader->use();
		resetPositionShader->setUniform("arraySize", arraySize);
		resetPositionShader->execute(1, 1, 1, 1, 1, 1);

		// FOURTH STEP: build tree back to first level and compute position of each bit
		downSweepShader->bindBuffers(std::vector<GLuint> { nBitsBufferID });
		downSweepShader->use();
		downSweepShader->setUniform("arraySize", arraySize);

		iteration = unsigned(threadCount.size()) - 2;
		while (iteration >= 0 && iteration < numExec)
		{
			downSweepShader->setUniform("iteration", iteration);
			downSweepShader->setUniform("numThreads", threadCount[iteration--]);
			downSweepShader->execute(numGroups2Log, 1, 1, maxGroupSize, 1, 1);
		}

		reallocatePositionShader->bindBuffers(std::vector<GLuint> { pBitsBufferID, nBitsBufferID, indicesBufferID_1, indicesBufferID_2 });
		reallocatePositionShader->use();
		reallocatePositionShader->setUniform("arraySize", arraySize);
		reallocatePositionShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);
	}

	glDeleteBuffers(1, &indicesBufferID_1);
	glDeleteBuffers(1, &pBitsBufferID);
	glDeleteBuffers(1, &nBitsBufferID);
	glDeleteBuffers(1, &positionBufferID);

	delete[] indices;

	return indicesBufferID_2;
}
