#include "stdafx.h"
#include "PointCloudBVH.h"

#include "Graphics/Core/ShaderList.h"
#include "Graphics/Core/VAO.h"

/// [Initialization of static attributes]

const GLuint PointCloudBVH::BVH_BUILDING_RADIUS = 100;

/// [Public methods]

PointCloudBVH::PointCloudBVH(std::vector<vec4>* points, EnvCamera* camera, const AABB& aabb)
	: _points(points), _aabb(aabb), _camera(camera), _cameraDirection(.0f), _normalEpsilon(.0f), _searchRadius(.0f)
{
	const EnvImage::ImageType imageType = camera->getImageType();

	_focalLength = EnvCamera::getFocalLength(imageType);
	_height = camera->getLocalPosition().z;
	_imageWidth = EnvCamera::getOriginalImageSize(imageType).x;
	_sensorWidth = EnvCamera::getWidthHeightMM(imageType).x;
}

PointCloudBVH::PointCloudBVH(std::vector<vec4>* points, std::vector<vec4>* normals, const AABB& aabb, const float searchRadius, const float normalEps, const vec3& cameraDirection, const float cameraHeight)
	: _points(points), _normals(normals), _aabb(aabb), _camera(nullptr), _focalLength(.0f), _height(.0f), _imageWidth(.0f), _sensorWidth(.0f)
{
	_cameraDirection = glm::normalize(cameraDirection) * cameraHeight;
	_normalEpsilon = normalEps;
	_searchRadius = searchRadius;
}

PointCloudBVH::~PointCloudBVH()
{
	delete _gpuData;
}

void PointCloudBVH::buildBVH()
{
	_gpuData = new BVHVolatileGPUData();

	this->aggregateSSBOData();
	this->generateBVH();
}

void PointCloudBVH::testVisibility(std::vector<uint32_t>& indices)
{
	ComputeShader* solveCollisionShader = ShaderList::getInstance()->getComputeShader(RendEnum::BVH_COLLISION_SPHERE);

	unsigned numCollisions = 0;
	const unsigned size = unsigned(_points->size());
	const unsigned clusterSize = size * 2 - 1;
	const unsigned numGroups = ComputeShader::getNumGroups(size);

	const GLuint triangleCollisionSSBO = ComputeShader::setWriteBuffer(GLuint(), size, GL_DYNAMIC_DRAW);
	const GLuint counterSSBO = ComputeShader::setReadBuffer(&numCollisions, 1, GL_DYNAMIC_DRAW);

	solveCollisionShader->use();
	solveCollisionShader->bindBuffers(std::vector<GLuint>{
		_gpuData->_clusterSSBO, _gpuData->_groupGeometrySSBO, triangleCollisionSSBO, counterSSBO
	});

	solveCollisionShader->setUniform("cameraPosition", _camera->getLocalPosition());
	solveCollisionShader->setUniform("numClusters", clusterSize);
	solveCollisionShader->setUniform("numPoints", size);
	solveCollisionShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	numCollisions = *ComputeShader::readData(counterSSBO, unsigned());

	unsigned* pIndices = ComputeShader::readData(triangleCollisionSSBO, unsigned());
	indices = std::vector<unsigned>(pIndices, pIndices + numCollisions);

	// Delete buffers
	GLuint toDeleteBuffers[] = { counterSSBO, triangleCollisionSSBO };
	glDeleteBuffers(sizeof(toDeleteBuffers) / sizeof(GLuint), toDeleteBuffers);
}

void PointCloudBVH::testVisibilityForGroundDetection(std::vector<uint32_t>& indices)
{
	ComputeShader* solveCollisionShader = ShaderList::getInstance()->getComputeShader(RendEnum::BVH_COLLISION_GROUND);

	unsigned numCollisions = 0;
	const unsigned size = unsigned(_points->size());
	const unsigned clusterSize = size * 2 - 1;
	const unsigned numGroups = ComputeShader::getNumGroups(size);

	const GLuint normalVectorSSBO = ComputeShader::setReadBuffer(*_normals, GL_STATIC_DRAW);
	const GLuint triangleCollisionSSBO = ComputeShader::setWriteBuffer(GLuint(), size, GL_DYNAMIC_DRAW);
	const GLuint counterSSBO = ComputeShader::setReadBuffer(&numCollisions, 1, GL_DYNAMIC_DRAW);
	const GLuint testSSBO = ComputeShader::setWriteBuffer(vec4(), size, GL_DYNAMIC_DRAW);

	solveCollisionShader->use();
	solveCollisionShader->bindBuffers(std::vector<GLuint>{
		_gpuData->_clusterSSBO, _gpuData->_groupGeometrySSBO, normalVectorSSBO, triangleCollisionSSBO, counterSSBO
	});

	solveCollisionShader->setUniform("cameraOrientation", _cameraDirection);
	solveCollisionShader->setUniform("groundOrientation", glm::normalize(_cameraDirection));
	solveCollisionShader->setUniform("normalEpsilon", _normalEpsilon);
	solveCollisionShader->setUniform("numClusters", clusterSize);
	solveCollisionShader->setUniform("numPoints", size);
	solveCollisionShader->setUniform("searchRadius", _searchRadius);
	solveCollisionShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	numCollisions = *ComputeShader::readData(counterSSBO, unsigned());

	unsigned* pIndices = ComputeShader::readData(triangleCollisionSSBO, unsigned());
	indices = std::vector<unsigned>(pIndices, pIndices + numCollisions);

	// Delete buffers
	GLuint toDeleteBuffers[] = { counterSSBO, normalVectorSSBO, triangleCollisionSSBO };
	glDeleteBuffers(sizeof(toDeleteBuffers) / sizeof(GLuint), toDeleteBuffers);
}

/// [Protected methods]

void PointCloudBVH::aggregateSSBOData()
{
	unsigned numPoints = unsigned(_points->size());

	_gpuData->_groupGeometrySSBO = ComputeShader::setReadBuffer(*_points, GL_STATIC_DRAW);

	const GLuint mortonCodes = this->calculateMortonCodes();
	const GLuint sortedIndices = this->sortFacesByMortonCode(mortonCodes);

	if (_camera)
	{
		this->buildClusterBuffer(sortedIndices);
	}
	else
	{
		this->buildClusterBuffer4CustomRadius(sortedIndices);
	}
}

void PointCloudBVH::buildClusterBuffer(const GLuint sortedFaces)
{
	ComputeShader* buildClusterShader = ShaderList::getInstance()->getComputeShader(RendEnum::BUILD_CLUSTER_BUFFER_PCL);

	const unsigned arraySize = unsigned(_points->size());
	const unsigned clusterSize = arraySize * 2 - 1;
	const int numGroups = ComputeShader::getNumGroups(arraySize);

	_gpuData->_clusterSSBO = ComputeShader::setWriteBuffer(Model3D::BVHCluster(), clusterSize);
	_gpuData->_tempClusterSSBO = ComputeShader::setWriteBuffer(Model3D::BVHCluster(), arraySize);

	buildClusterShader->bindBuffers(std::vector<GLuint> { _gpuData->_groupGeometrySSBO, sortedFaces, _gpuData->_clusterSSBO, _gpuData->_tempClusterSSBO });
	buildClusterShader->use();
	buildClusterShader->setUniform("arraySize", arraySize);
	buildClusterShader->setUniform("cameraHeight", _height);
	buildClusterShader->setUniform("focalLength", _focalLength);
	buildClusterShader->setUniform("imageWidth", _imageWidth);
	buildClusterShader->setUniform("sensorWidth", _sensorWidth);
	buildClusterShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	glDeleteBuffers(1, &sortedFaces);
}

void PointCloudBVH::buildClusterBuffer4CustomRadius(const GLuint sortedFaces)
{
	ComputeShader* buildClusterShader = ShaderList::getInstance()->getComputeShader(RendEnum::BUILD_CLUSTER_BUFFER_PCL_CUSTOM_RADIUS);

	const unsigned arraySize = unsigned(_points->size());
	const unsigned clusterSize = arraySize * 2 - 1;
	const int numGroups = ComputeShader::getNumGroups(arraySize);

	_gpuData->_clusterSSBO = ComputeShader::setWriteBuffer(Model3D::BVHCluster(), clusterSize);
	_gpuData->_tempClusterSSBO = ComputeShader::setWriteBuffer(Model3D::BVHCluster(), arraySize);

	buildClusterShader->bindBuffers(std::vector<GLuint> { _gpuData->_groupGeometrySSBO, sortedFaces, _gpuData->_clusterSSBO, _gpuData->_tempClusterSSBO });
	buildClusterShader->use();
	buildClusterShader->setUniform("arraySize", arraySize);
	buildClusterShader->setUniform("searchRadius", _searchRadius);
	buildClusterShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	glDeleteBuffers(1, &sortedFaces);
}

GLuint PointCloudBVH::calculateMortonCodes()
{
	ComputeShader* computeMortonShader = ShaderList::getInstance()->getComputeShader(RendEnum::COMPUTE_MORTON_CODES_PCL);

	const unsigned arraySize = unsigned(_points->size());
	const int numGroups = ComputeShader::getNumGroups(arraySize);
	const GLuint mortonCodeBuffer = ComputeShader::setWriteBuffer(unsigned(), arraySize);

	computeMortonShader->bindBuffers(std::vector<GLuint> { _gpuData->_groupGeometrySSBO, mortonCodeBuffer });
	computeMortonShader->use();
	computeMortonShader->setUniform("arraySize", arraySize);
	computeMortonShader->setUniform("sceneMaxBoundary", _aabb.max());
	computeMortonShader->setUniform("sceneMinBoundary", _aabb.min());
	computeMortonShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	return mortonCodeBuffer;
}

void PointCloudBVH::generateBVH()
{
	// BVH generation
	const unsigned radius = BVH_BUILDING_RADIUS;

	ComputeShader* findNeighborShader = ShaderList::getInstance()->getComputeShader(RendEnum::FIND_BEST_NEIGHBOR_PCL);
	ComputeShader* clusterMergingShader = ShaderList::getInstance()->getComputeShader(RendEnum::CLUSTER_MERGING_PCL);
	ComputeShader* reallocClustersShader = ShaderList::getInstance()->getComputeShader(RendEnum::REALLOCATE_CLUSTERS_PCL);
	ComputeShader* endLoopCompShader = ShaderList::getInstance()->getComputeShader(RendEnum::END_LOOP_COMPUTATIONS_PCL);

	// Prefix scan
	ComputeShader* reduceShader = ShaderList::getInstance()->getComputeShader(RendEnum::REDUCE_PREFIX_SCAN);
	ComputeShader* downSweepShader = ShaderList::getInstance()->getComputeShader(RendEnum::DOWN_SWEEP_PREFIX_SCAN);
	ComputeShader* resetPositionShader = ShaderList::getInstance()->getComputeShader(RendEnum::RESET_LAST_POSITION_PREFIX_SCAN);

	// Compute shader execution data: groups and iteration control
	int numGroups, numGroups2Log;
	unsigned arraySize = unsigned(_points->size()), startIndex = arraySize, finishBit = 0, iteration, numExec, numThreads, startThreads;
	const int maxGroupSize = ComputeShader::getMaxGroupSize();

	// Compact cluster buffer support
	GLuint* currentPosBufferOut = new GLuint[arraySize];
	std::iota(currentPosBufferOut, currentPosBufferOut + arraySize, 0);

	GLuint coutBuffer = _gpuData->_tempClusterSSBO;												// Swapped during loop => not const
	GLuint cinBuffer = ComputeShader::setWriteBuffer(Model3D::BVHCluster(), arraySize);
	GLuint inCurrentPosition = ComputeShader::setWriteBuffer(GLuint(), arraySize);				// Position of compact buffer where a cluster is saved
	GLuint outCurrentPosition = ComputeShader::setReadBuffer(currentPosBufferOut, arraySize);
	const GLuint neighborIndex = ComputeShader::setWriteBuffer(GLuint(), arraySize);			// Nearest neighbor search	
	const GLuint prefixScan = ComputeShader::setWriteBuffer(GLuint(), arraySize);				// Final position of each valid cluster for the next loop iteration
	const GLuint validCluster = ComputeShader::setWriteBuffer(GLuint(), arraySize);				// Clusters which takes part of next loop iteration
	const GLuint mergedCluster = ComputeShader::setWriteBuffer(GLuint(), arraySize);			// A merged cluster is always valid, but the opposite situation is not fitting
	const GLuint numNodesCount = ComputeShader::setReadData(arraySize);							// Number of currently added nodes, which increases as the clusters are merged
	const GLuint arraySizeCount = ComputeShader::setWriteBuffer(GLuint(), 1);

	while (arraySize > 1)
	{
		// Binary tree and whole array group sizes and iteration boundaries
		numGroups = ComputeShader::getNumGroups(arraySize);
		startThreads = unsigned(std::ceil(arraySize / 2.0f));
		numExec = unsigned(std::ceil(std::log2(arraySize)));
		numGroups2Log = ComputeShader::getNumGroups(startThreads);

		std::vector<GLuint> threadCount{ startThreads };				// Thread sizes are repeated on reduce and sweep down phases
		threadCount.reserve(numExec);

		std::swap(coutBuffer, cinBuffer);
		std::swap(inCurrentPosition, outCurrentPosition);

		findNeighborShader->bindBuffers(std::vector<GLuint>{ cinBuffer, neighborIndex });
		findNeighborShader->use();
		findNeighborShader->setUniform("arraySize", arraySize);
		findNeighborShader->setUniform("radius", radius);
		findNeighborShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

		clusterMergingShader->bindBuffers(std::vector<GLuint>{
			cinBuffer, _gpuData->_clusterSSBO, neighborIndex, validCluster, mergedCluster,
				prefixScan, inCurrentPosition, numNodesCount });
		clusterMergingShader->use();
		clusterMergingShader->setUniform("arraySize", arraySize);
		clusterMergingShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

		// FIRST STEP: build a binary tree with a summatory of the array
		reduceShader->bindBuffers(std::vector<GLuint> { prefixScan });
		reduceShader->use();
		reduceShader->setUniform("arraySize", arraySize);

		iteration = 0;
		while (iteration < numExec)
		{
			numThreads = threadCount[threadCount.size() - 1];

			reduceShader->setUniform("iteration", iteration++);
			reduceShader->setUniform("numThreads", numThreads);
			reduceShader->execute(numGroups2Log, 1, 1, maxGroupSize, 1, 1);

			threadCount.push_back(GLuint(std::ceil(numThreads / 2.0f)));
		}

		// SECOND STEP: set last position to zero, its faster to do it in GPU than retrieve the array in CPU, modify and write it again to GPU
		resetPositionShader->bindBuffers(std::vector<GLuint> { prefixScan });
		resetPositionShader->use();
		resetPositionShader->setUniform("arraySize", arraySize);
		resetPositionShader->execute(1, 1, 1, 1, 1, 1);

		// THIRD STEP: build tree back to first level and compute final summatory
		downSweepShader->bindBuffers(std::vector<GLuint> { prefixScan });
		downSweepShader->use();
		downSweepShader->setUniform("arraySize", arraySize);

		iteration = unsigned(threadCount.size()) - 2;
		while (iteration >= 0 && iteration < numExec)
		{
			downSweepShader->setUniform("iteration", iteration);
			downSweepShader->setUniform("numThreads", threadCount[iteration--]);
			downSweepShader->execute(numGroups2Log, 1, 1, maxGroupSize, 1, 1);
		}

		reallocClustersShader->bindBuffers(std::vector<GLuint>{ cinBuffer, coutBuffer, validCluster, prefixScan, inCurrentPosition, outCurrentPosition });
		reallocClustersShader->use();
		reallocClustersShader->setUniform("arraySize", arraySize);
		reallocClustersShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

		// Updates cluster size
		endLoopCompShader->bindBuffers(std::vector<GLuint>{ arraySizeCount, prefixScan, validCluster });
		endLoopCompShader->use();
		endLoopCompShader->setUniform("arraySize", arraySize);
		endLoopCompShader->execute(1, 1, 1, 1, 1, 1);

		arraySize = endLoopCompShader->readData(arraySizeCount, GLuint())[0];
	}

	Model3D::BVHCluster* clusterData = ComputeShader::readData(_gpuData->_clusterSSBO, Model3D::BVHCluster());
	std::vector<Model3D::BVHCluster> cluster = std::vector<Model3D::BVHCluster>(clusterData, clusterData + _points->size());

	// Free buffers from GPU
	GLuint toDeleteBuffers[] = { coutBuffer, cinBuffer, inCurrentPosition, outCurrentPosition, neighborIndex,
								 prefixScan, validCluster, mergedCluster, numNodesCount, arraySizeCount };
	glDeleteBuffers(sizeof(toDeleteBuffers) / sizeof(GLuint), toDeleteBuffers);

	// Free dynamic memory
	delete[] currentPosBufferOut;
}

GLuint PointCloudBVH::sortFacesByMortonCode(const GLuint mortonCodes)
{
	ComputeShader* bitMaskShader = ShaderList::getInstance()->getComputeShader(RendEnum::BIT_MASK_RADIX_SORT);
	ComputeShader* reduceShader = ShaderList::getInstance()->getComputeShader(RendEnum::REDUCE_PREFIX_SCAN);
	ComputeShader* downSweepShader = ShaderList::getInstance()->getComputeShader(RendEnum::DOWN_SWEEP_PREFIX_SCAN);
	ComputeShader* resetPositionShader = ShaderList::getInstance()->getComputeShader(RendEnum::RESET_LAST_POSITION_PREFIX_SCAN);
	ComputeShader* reallocatePositionShader = ShaderList::getInstance()->getComputeShader(RendEnum::REALLOCATE_RADIX_SORT);

	const unsigned numBits = 30;			// 10 bits per coordinate (3D)
	unsigned arraySize = unsigned(_points->size());
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

		bitMaskShader->bindBuffers(std::vector<GLuint> { mortonCodes, indicesBufferID_1, pBitsBufferID, nBitsBufferID });
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