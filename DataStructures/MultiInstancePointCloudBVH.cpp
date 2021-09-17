#include "stdafx.h"
#include "MultiInstancePointCloudBVH.h"

#include "Graphics/Core/ShaderList.h"
#include "Graphics/Core/VAO.h"

/// [Initialization of static attributes]

const GLuint MultiInstancePointCloudBVH::BVH_BUILDING_RADIUS = 20;

/// [Public methods]

MultiInstancePointCloudBVH::MultiInstancePointCloudBVH(const unsigned arraySize)
	: _groupGeometrySSBO(-1)
{
	_arraySizeCount		= ComputeShader::setWriteBuffer(GLuint(), 1);
	_cinBuffer			= ComputeShader::setWriteBuffer(Model3D::BVHCluster(), arraySize);
	_clusterSSBO		= ComputeShader::setWriteBuffer(Model3D::BVHCluster(), arraySize * 2 - 1);
	_inCurrentPosition	= ComputeShader::setWriteBuffer(GLuint(), arraySize);
	_indicesBufferID_1	= ComputeShader::setWriteBuffer(GLuint(), arraySize);
	_mergedCluster		= ComputeShader::setWriteBuffer(GLuint(), arraySize);		
	_mortonCodeBuffer	= ComputeShader::setWriteBuffer(GLuint(), arraySize);
	_nBitsBufferID		= ComputeShader::setWriteBuffer(GLuint(), arraySize);
	_neighborIndex		= ComputeShader::setWriteBuffer(GLuint(), arraySize);				
	_numNodesCount		= ComputeShader::setReadData(arraySize);							
	_outCurrentPosition = ComputeShader::setWriteBuffer(GLuint(), 1);
	_pBitsBufferID		= ComputeShader::setWriteBuffer(GLuint(), arraySize);
	_positionBufferID	= ComputeShader::setWriteBuffer(GLuint(), arraySize);
	_prefixScan			= ComputeShader::setWriteBuffer(GLuint(), arraySize);	
	_tempClusterSSBO	= ComputeShader::setWriteBuffer(Model3D::BVHCluster(), arraySize);
	_triangleCollisionSSBO = ComputeShader::setWriteBuffer(GLuint(), arraySize, GL_DYNAMIC_DRAW);
	_validCluster		= ComputeShader::setWriteBuffer(GLuint(), arraySize);			

	{
		_indices = new GLuint[arraySize];
		std::iota(_indices, _indices + arraySize, 0);
	}
}

MultiInstancePointCloudBVH::~MultiInstancePointCloudBVH()
{
	GLuint toDeleteBuffers[] = {
		_arraySizeCount, _cinBuffer, _clusterSSBO, _inCurrentPosition, _indicesBufferID_1, _mergedCluster, _mortonCodeBuffer, _nBitsBufferID, _neighborIndex, _numNodesCount,
		_outCurrentPosition, _pBitsBufferID, _positionBufferID, _prefixScan, _tempClusterSSBO, _triangleCollisionSSBO, _validCluster
	};
	glDeleteBuffers(sizeof(toDeleteBuffers) / sizeof(GLuint), toDeleteBuffers);

	delete[] _indices;
}

void MultiInstancePointCloudBVH::buildBVH()
{
	this->aggregateSSBOData();
	this->generateBVH();
}

void MultiInstancePointCloudBVH::reserveSpace(std::vector<vec4>* points, EnvCamera* camera, const AABB& aabb)
{
	_aabb = aabb;
	_camera = camera;
	_points = points;

	{
		const EnvImage::ImageType imageType = camera->getImageType();

		_focalLength = EnvCamera::getFocalLength(imageType);
		_height = camera->getLocalPosition().z;
		_imageWidth = EnvCamera::getOriginalImageSize(imageType).x;
		_sensorWidth = EnvCamera::getWidthHeightMM(imageType).x;
	}
}

void MultiInstancePointCloudBVH::testVisibility(std::vector<uint32_t>& indices)
{
	ComputeShader* solveCollisionShader = ShaderList::getInstance()->getComputeShader(RendEnum::BVH_COLLISION_SPHERE);

	unsigned numCollisions = 0;
	const unsigned size = unsigned(_points->size());
	const unsigned clusterSize = size * 2 - 1;
	const unsigned numGroups = ComputeShader::getNumGroups(size);

	const GLuint counterSSBO = ComputeShader::setReadBuffer(&numCollisions, 1, GL_DYNAMIC_DRAW);

	solveCollisionShader->use();
	solveCollisionShader->bindBuffers(std::vector<GLuint>{
		_clusterSSBO, _groupGeometrySSBO, _triangleCollisionSSBO, counterSSBO
	});

	solveCollisionShader->setUniform("cameraPosition", _camera->getLocalPosition());
	solveCollisionShader->setUniform("numClusters", clusterSize);
	solveCollisionShader->setUniform("numPoints", size);
	solveCollisionShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	numCollisions = *ComputeShader::readData(counterSSBO, unsigned());

	unsigned* pIndices = ComputeShader::readData(_triangleCollisionSSBO, unsigned());
	indices = std::vector<unsigned>(pIndices, pIndices + numCollisions);

	// Delete buffers
	GLuint toDeleteBuffers[] = { counterSSBO, _groupGeometrySSBO };
	glDeleteBuffers(sizeof(toDeleteBuffers) / sizeof(GLuint), toDeleteBuffers);
}

/// [Protected methods]

void MultiInstancePointCloudBVH::aggregateSSBOData()
{
	unsigned numPoints = unsigned(_points->size());

	_groupGeometrySSBO = ComputeShader::setReadBuffer(*_points, GL_DYNAMIC_DRAW);

	const GLuint mortonCodes = this->calculateMortonCodes();
	const GLuint sortedIndices = this->sortFacesByMortonCode(mortonCodes);

	this->buildClusterBuffer(sortedIndices);
}

void MultiInstancePointCloudBVH::buildClusterBuffer(const GLuint sortedFaces)
{
	ComputeShader* buildClusterShader = ShaderList::getInstance()->getComputeShader(RendEnum::BUILD_CLUSTER_BUFFER_PCL);

	const unsigned arraySize = unsigned(_points->size());
	const unsigned clusterSize = arraySize * 2 - 1;
	const int numGroups = ComputeShader::getNumGroups(arraySize);

	buildClusterShader->bindBuffers(std::vector<GLuint> { _groupGeometrySSBO, sortedFaces, _clusterSSBO, _tempClusterSSBO });
	buildClusterShader->use();
	buildClusterShader->setUniform("arraySize", arraySize);
	buildClusterShader->setUniform("cameraHeight", _height);
	buildClusterShader->setUniform("focalLength", _focalLength);
	buildClusterShader->setUniform("imageWidth", _imageWidth);
	buildClusterShader->setUniform("sensorWidth", _sensorWidth);
	buildClusterShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	glDeleteBuffers(1, &sortedFaces);
}

GLuint MultiInstancePointCloudBVH::calculateMortonCodes()
{
	ComputeShader* computeMortonShader = ShaderList::getInstance()->getComputeShader(RendEnum::COMPUTE_MORTON_CODES_PCL);

	const unsigned arraySize = unsigned(_points->size());
	const int numGroups = ComputeShader::getNumGroups(arraySize);

	computeMortonShader->bindBuffers(std::vector<GLuint> { _groupGeometrySSBO, _mortonCodeBuffer });
	computeMortonShader->use();
	computeMortonShader->setUniform("arraySize", arraySize);
	computeMortonShader->setUniform("sceneMaxBoundary", _aabb.max());
	computeMortonShader->setUniform("sceneMinBoundary", _aabb.min());
	computeMortonShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	return _mortonCodeBuffer;
}

void MultiInstancePointCloudBVH::generateBVH()
{
	// BVH generation
	const unsigned radius = BVH_BUILDING_RADIUS;

	ComputeShader* findNeighborShader		= ShaderList::getInstance()->getComputeShader(RendEnum::FIND_BEST_NEIGHBOR_PCL);
	ComputeShader* clusterMergingShader		= ShaderList::getInstance()->getComputeShader(RendEnum::CLUSTER_MERGING_PCL);
	ComputeShader* reallocClustersShader	= ShaderList::getInstance()->getComputeShader(RendEnum::REALLOCATE_CLUSTERS_PCL);
	ComputeShader* endLoopCompShader		= ShaderList::getInstance()->getComputeShader(RendEnum::END_LOOP_COMPUTATIONS_PCL);

	// Prefix scan
	ComputeShader* reduceShader				= ShaderList::getInstance()->getComputeShader(RendEnum::REDUCE_PREFIX_SCAN);
	ComputeShader* downSweepShader			= ShaderList::getInstance()->getComputeShader(RendEnum::DOWN_SWEEP_PREFIX_SCAN);
	ComputeShader* resetPositionShader		= ShaderList::getInstance()->getComputeShader(RendEnum::RESET_LAST_POSITION_PREFIX_SCAN);

	// Compute shader execution data: groups and iteration control
	int numGroups, numGroups2Log;
	unsigned arraySize		= unsigned(_points->size()), startIndex = arraySize, finishBit = 0, iteration, numExec, numThreads, startThreads;
	const int maxGroupSize	= ComputeShader::getMaxGroupSize();

	GLuint coutBuffer			= _tempClusterSSBO;										// Swapped during loop => not const
	GLuint cinBuffer			= _cinBuffer;
	GLuint inCurrentPosition	= _inCurrentPosition;
	GLuint outCurrentPosition	= ComputeShader::setReadBuffer(_indices, arraySize);
	GLuint outBufferID			= outCurrentPosition;
	const GLuint numNodesCount	= ComputeShader::setReadData(arraySize);							// Number of currently added nodes, which increases as the clusters are merged

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

		findNeighborShader->bindBuffers(std::vector<GLuint>{ cinBuffer, _neighborIndex });
		findNeighborShader->use();
		findNeighborShader->setUniform("arraySize", arraySize);
		findNeighborShader->setUniform("radius", radius);
		findNeighborShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

		clusterMergingShader->bindBuffers(std::vector<GLuint>{
			cinBuffer, _clusterSSBO, _neighborIndex, _validCluster, _mergedCluster,
			_prefixScan, inCurrentPosition, numNodesCount
		});
		clusterMergingShader->use();
		clusterMergingShader->setUniform("arraySize", arraySize);
		clusterMergingShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

		// FIRST STEP: build a binary tree with a summatory of the array
		reduceShader->bindBuffers(std::vector<GLuint> { _prefixScan });
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
		resetPositionShader->bindBuffers(std::vector<GLuint> { _prefixScan });
		resetPositionShader->use();
		resetPositionShader->setUniform("arraySize", arraySize);
		resetPositionShader->execute(1, 1, 1, 1, 1, 1);

		// THIRD STEP: build tree back to first level and compute final summatory
		downSweepShader->bindBuffers(std::vector<GLuint> { _prefixScan });
		downSweepShader->use();
		downSweepShader->setUniform("arraySize", arraySize);

		iteration = unsigned(threadCount.size()) - 2;
		while (iteration >= 0 && iteration < numExec)
		{
			downSweepShader->setUniform("iteration", iteration);
			downSweepShader->setUniform("numThreads", threadCount[iteration--]);
			downSweepShader->execute(numGroups2Log, 1, 1, maxGroupSize, 1, 1);
		}

		reallocClustersShader->bindBuffers(std::vector<GLuint>{ cinBuffer, coutBuffer, _validCluster, _prefixScan, inCurrentPosition, outCurrentPosition });
		reallocClustersShader->use();
		reallocClustersShader->setUniform("arraySize", arraySize);
		reallocClustersShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

		// Updates cluster size
		endLoopCompShader->bindBuffers(std::vector<GLuint>{ _arraySizeCount, _prefixScan, _validCluster });
		endLoopCompShader->use();
		endLoopCompShader->setUniform("arraySize", arraySize);
		endLoopCompShader->execute(1, 1, 1, 1, 1, 1);

		arraySize = endLoopCompShader->readData(_arraySizeCount, GLuint())[0];
	}

	glDeleteBuffers(1, &outBufferID);
}

GLuint MultiInstancePointCloudBVH::sortFacesByMortonCode(const GLuint mortonCodes)
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

	// Binary tree parameters
	const unsigned startThreads = unsigned(std::ceil(arraySize / 2.0f));
	const unsigned numExec = unsigned(std::ceil(std::log2(arraySize)));
	const unsigned numGroups2Log = unsigned(ComputeShader::getNumGroups(startThreads));
	unsigned numThreads = 0, iteration;

	GLuint indicesBufferID_2;
	indicesBufferID_2 = ComputeShader::setReadBuffer(_indices, arraySize);					// Substitutes indicesBufferID_1 for the next iteration

	while (currentBits < numBits)
	{
		std::vector<GLuint> threadCount{ startThreads };
		threadCount.reserve(numExec);

		std::swap(_indicesBufferID_1, indicesBufferID_2);							// indicesBufferID_2 is initialized with indices cause it's swapped here

		// FIRST STEP: BIT MASK, check if a morton code gives zero or one for a certain mask (iteration)
		unsigned bitMask = 1 << currentBits++;

		bitMaskShader->bindBuffers(std::vector<GLuint> { mortonCodes, _indicesBufferID_1, _pBitsBufferID, _nBitsBufferID });
		bitMaskShader->use();
		bitMaskShader->setUniform("arraySize", arraySize);
		bitMaskShader->setUniform("bitMask", bitMask);
		bitMaskShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

		// SECOND STEP: build a binary tree with a summatory of the array
		reduceShader->bindBuffers(std::vector<GLuint> { _nBitsBufferID });
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
		resetPositionShader->bindBuffers(std::vector<GLuint> { _nBitsBufferID });
		resetPositionShader->use();
		resetPositionShader->setUniform("arraySize", arraySize);
		resetPositionShader->execute(1, 1, 1, 1, 1, 1);

		// FOURTH STEP: build tree back to first level and compute position of each bit
		downSweepShader->bindBuffers(std::vector<GLuint> { _nBitsBufferID });
		downSweepShader->use();
		downSweepShader->setUniform("arraySize", arraySize);

		iteration = unsigned(threadCount.size()) - 2;
		while (iteration >= 0 && iteration < numExec)
		{
			downSweepShader->setUniform("iteration", iteration);
			downSweepShader->setUniform("numThreads", threadCount[iteration--]);
			downSweepShader->execute(numGroups2Log, 1, 1, maxGroupSize, 1, 1);
		}

		reallocatePositionShader->bindBuffers(std::vector<GLuint> { _pBitsBufferID, _nBitsBufferID, _indicesBufferID_1, indicesBufferID_2 });
		reallocatePositionShader->use();
		reallocatePositionShader->setUniform("arraySize", arraySize);
		reallocatePositionShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);
	}

	return indicesBufferID_2;
}