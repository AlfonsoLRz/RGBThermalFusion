#include "stdafx.h"
#include "EnvPointCloud.h"

#include <filesystem>
#include "Graphics/Application/TextureList.h"
#include "Graphics/Core/ShaderList.h"
#include "Graphics/Core/VAO.h"
#include "ImportedLibraries/tinyply.h"

// Initialization of static attributes
const std::string	EnvPointCloud::SECONDARY_BINARY_EXTENSION = "-secondary";
const float			EnvPointCloud::TARGET_SCALE_X = 20.0f;
const std::string	EnvPointCloud::WRITE_POINT_CLOUD_FOLDER = "PointClouds/";

/// Public methods

EnvPointCloud::EnvPointCloud(const std::string& filename, const bool useBinary, const mat4& modelMatrix) : 
	Model3D(modelMatrix, 1), _filename(filename), _meanTemperature(.0f), _offset(.0f), _useBinary(useBinary), _temperatureRange(FLT_MAX, FLT_MIN),
	_q1(.0f), _q3(.0f), _interquartileRange(.0f)
{
}

EnvPointCloud::~EnvPointCloud()
{
}

void EnvPointCloud::computeQuartiles()
{
	GLuint startQ1, endQ1, startQ3, endQ3, medianQ1Index, medianQ2Index, medianQ3Index;
	std::vector<float> sortedTemperature;

	this->sortTemperature(sortedTemperature);

	startQ1 = 0;
	endQ3 = sortedTemperature.size() - 1;

	// Compute each quartile by splitting the vector
	if (sortedTemperature.size() % 2)		// Odd
	{
		medianQ2Index = std::floor(sortedTemperature.size() / 2.0f);
		endQ1 = startQ3 = medianQ2Index;

		_q1 = sortedTemperature[(endQ1 + startQ1) / 2];
		_q3 = sortedTemperature[(endQ3 + startQ3) / 2];
	}
	else       // Even
	{
		endQ1 = sortedTemperature.size() / 2.0f - 1;
		startQ3 = endQ1 + 1;
		
		_q1 = (sortedTemperature[std::floor((endQ1 + startQ1) / 2.0f)] + sortedTemperature[std::ceil((endQ1 + startQ1) / 2.0f)]) / 2.0f;
		_q3 = (sortedTemperature[std::floor((endQ3 + startQ3) / 2.0f)] + sortedTemperature[std::ceil((endQ3 + startQ3) / 2.0f)]) / 2.0f;
	}

	_interquartileRange = _q3 - _q1;
}

bool EnvPointCloud::load(const mat4& modelMatrix)
{
	if (!_loaded)
	{
		bool success = false, binaryExists = false;

		if (_useBinary && (binaryExists = std::filesystem::exists(_filename + BINARY_EXTENSION)))
		{
			success = this->loadModelFromBinaryFile();
		}

		if (!success)
		{
			success = this->loadModelFromPLY(modelMatrix);
		}

		std::cout << "Number of Points: " << _points.size() << std::endl;

		if (success)
		{
			this->computeCloudData();
			this->setVAOData();

			if (!binaryExists)
			{
				this->writeToBinary(_filename + BINARY_EXTENSION);
			}
		}

		_loaded = true;
		
		return true;
	}

	return false;
}

bool EnvPointCloud::loadSecondaryColorsFromBinary()
{
	std::ifstream fin(_filename + SECONDARY_BINARY_EXTENSION + BINARY_EXTENSION, std::ios::in | std::ios::binary);
	if (!fin.is_open())
	{
		return false;
	}

	size_t numColors;
	
	for (int colorIdx = 0; colorIdx < NUM_COLORS; ++colorIdx)
	{
		fin.read((char*)&numColors, sizeof(size_t));
		_secondaryColor[colorIdx].resize(numColors);
		fin.read((char*)&_secondaryColor[colorIdx][0], numColors * sizeof(vec4));
		this->loadSecondaryColor(static_cast<EnvPointCloud::PointColorTypes>(colorIdx));
	}
	fin.read((char*)&_temperatureRange, sizeof(vec2));
	fin.read((char*)&_meanTemperature, sizeof(float));

	std::cout << "Temperature Range: " << _temperatureRange.x << ", " << _temperatureRange.y << std::endl;
	std::cout << "Mean Temperature: " << _meanTemperature << std::endl;
	
	fin.close();

	return true;
}

void EnvPointCloud::setNormals(std::vector<vec4>& normals)
{
	VAO* vao = _modelComp[0]->_vao;

	if (vao)
	{
		vao->setVBOData(RendEnum::VBO_NORMAL, normals, GL_STATIC_DRAW);
	}
}

void EnvPointCloud::setPointClass(const std::vector<unsigned>& pointClass)
{
	VAO* vao = _modelComp[0]->_vao;

	if (vao)
	{
		const GLuint slotIndex = vao->defineVBO(RendEnum::VBO_CLASS, unsigned(), GL_FLOAT);
		vao->setVBOData(RendEnum::VBO_CLASS, pointClass, GL_STATIC_DRAW);
	}
}

void EnvPointCloud::setSecondaryColor(const PointColorTypes colorType, std::vector<vec4>& colors)
{
	_secondaryColor[colorType] = std::move(colors);
	this->loadSecondaryColor(colorType);
}

float EnvPointCloud::queryTemperature(const vec4& screenColor)
{
	if (_temperatureRange.x > _temperatureRange.y) return FLT_MAX;
	
	return screenColor.x * (_temperatureRange.y - _temperatureRange.x) + _temperatureRange.x;
}

bool EnvPointCloud::writePointCloud(const std::string& filename, const bool ascii)
{
	std::thread writePointCloudThread(&EnvPointCloud::threadedWritePointCloud, this, filename, ascii);
	writePointCloudThread.detach();

	return true;
}

bool EnvPointCloud::writeSecondaryColorsToBinary()
{
	std::ofstream fout(_filename + SECONDARY_BINARY_EXTENSION + BINARY_EXTENSION, std::ios::out | std::ios::binary);
	if (!fout.is_open())
	{
		return false;
	}

	for (int colorIdx = 0; colorIdx < NUM_COLORS; ++colorIdx)
	{
		const size_t numColors = _secondaryColor[colorIdx].size();
		fout.write((char*)&numColors, sizeof(size_t));
		fout.write((char*)&_secondaryColor[colorIdx][0], numColors * sizeof(vec4));
	}
	fout.write((char*)&_temperatureRange[0], sizeof(vec2));
	fout.write((char*)&_meanTemperature, sizeof(float));

	fout.close();

	return true;
}

/// [Protected methods]

void EnvPointCloud::computeCloudData()
{
	ModelComponent* modelComp = _modelComp[0];

	// Fill point cloud indices with iota
	modelComp->_pointCloud.resize(_points.size());
	std::iota(modelComp->_pointCloud.begin(), modelComp->_pointCloud.end(), 0);
}

bool EnvPointCloud::loadModelFromBinaryFile()
{
	return this->readBinary(_filename + BINARY_EXTENSION, _modelComp);
}

bool EnvPointCloud::loadModelFromPLY(const mat4& modelMatrix)
{
	std::unique_ptr<std::istream> fileStream;
	std::vector<uint8_t> byteBuffer;
	std::shared_ptr<tinyply::PlyData> plyPoints, plyColors;
	unsigned baseIndex;
	float* pointsRaw;
	uint8_t* colorsRaw;

	try
	{
		const std::string filename = _filename + PLY_EXTENSION;
		fileStream.reset(new std::ifstream(filename, std::ios::binary));

		if (!fileStream || fileStream->fail()) return false;

		fileStream->seekg(0, std::ios::end);
		const float size_mb = fileStream->tellg() * float(1e-6);
		fileStream->seekg(0, std::ios::beg);

		tinyply::PlyFile file;
		file.parse_header(*fileStream);

		try { plyPoints = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
		catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }
		
		try { plyColors = file.request_properties_from_element("vertex", { "red", "green", "blue" }); }
		catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

		file.read(*fileStream);

		{
			const size_t numPoints = plyPoints->count;
			const size_t numPointsBytes = numPoints * 4 * 3;

			const size_t numColors = plyColors->count;
			const size_t numColorsBytes = numColors * 1 * 3;

			// Allocate space
			_points.resize(numPoints); _rgb.resize(numPoints);
			pointsRaw = new float[numPoints * 3];
			colorsRaw = new uint8_t[numColors * 3];

			std::memcpy(pointsRaw, plyPoints->buffer.get(), numPointsBytes);
			std::memcpy(colorsRaw, plyColors->buffer.get(), numColorsBytes);

			for (unsigned ind = 0; ind < numPoints; ++ind)
			{
				baseIndex = ind * 3;

				_points[ind] = vec4(pointsRaw[baseIndex], pointsRaw[baseIndex + 1], pointsRaw[baseIndex + 2], 1.0f);
				_rgb[ind] = vec3(colorsRaw[baseIndex] / 255.0f, colorsRaw[baseIndex + 1] / 255.0f, colorsRaw[baseIndex + 2] / 255.0f);
				_aabb.update(vec3(_points[ind]));
			}
		}
	}
	catch (const std::exception & e)
	{
		std::cerr << "Caught tinyply exception: " << e.what() << std::endl;

		return false;
	}

	return true;
}

bool EnvPointCloud::loadSecondaryColor(const PointColorTypes colorType)
{
	VAO* vao = _modelComp[0]->_vao;

	if (_secondaryColor[colorType].size() > 0 && vao)
	{
		vao->defineVBO(static_cast<RendEnum::VBOTypes>(RendEnum::VBO_COLOR_02 + colorType), vec4(.0f), GL_FLOAT);
		vao->setVBOData(static_cast<RendEnum::VBOTypes>(RendEnum::VBO_COLOR_02 + colorType), _secondaryColor[colorType], GL_STATIC_DRAW);

		return true;
	}

	return false;
}

bool EnvPointCloud::readBinary(const std::string& filename, const std::vector<Model3D::ModelComponent*>& modelComp)
{
	std::ifstream fin(filename, std::ios::in | std::ios::binary);
	if (!fin.is_open())
	{
		return false;
	}

	size_t numPoints;

	fin.read((char*)&numPoints, sizeof(size_t));
	_points.resize(numPoints); _rgb.resize(numPoints);
	fin.read((char*)&_points[0], numPoints * sizeof(vec4));
	fin.read((char*)&_rgb[0], numPoints * sizeof(vec3));
	fin.read((char*)&_aabb, sizeof(AABB));

	fin.close();

	return true;
}

void EnvPointCloud::setVAOData()
{
	VAO* vao = new VAO(false);
	ModelComponent* modelComp = _modelComp[0];

	// Refresh point cloud length
	modelComp->_topologyIndicesLength[RendEnum::IBO_POINT_CLOUD] = unsigned(modelComp->_pointCloud.size());

	vao->defineVBO(RendEnum::VBO_COLOR_01, vec3(.0f), GL_FLOAT);
	vao->setVBOData(RendEnum::VBO_POSITION, _points, GL_STATIC_DRAW);
	vao->setVBOData(RendEnum::VBO_COLOR_01, _rgb, GL_STATIC_DRAW);
	vao->setIBOData(RendEnum::IBO_POINT_CLOUD, modelComp->_pointCloud);

	modelComp->_vao = vao;
}

void EnvPointCloud::sortTemperature(std::vector<float>& sortedTemperature)
{
	static const float TEMPERATURE_MULTIPLICATOR = 100000.0f;
	std::vector<GLuint> temperature;

	for (vec4& color: _secondaryColor[PointColorTypes::TEMPERATURE])
	{
		if (color.w > glm::epsilon<float>()) temperature.push_back(color.y * TEMPERATURE_MULTIPLICATOR);
	}

	// Sort uint values
	{
		ComputeShader* bitMaskShader = ShaderList::getInstance()->getComputeShader(RendEnum::BIT_MASK_RADIX_SORT);
		ComputeShader* reduceShader = ShaderList::getInstance()->getComputeShader(RendEnum::REDUCE_PREFIX_SCAN);
		ComputeShader* downSweepShader = ShaderList::getInstance()->getComputeShader(RendEnum::DOWN_SWEEP_PREFIX_SCAN);
		ComputeShader* resetPositionShader = ShaderList::getInstance()->getComputeShader(RendEnum::RESET_LAST_POSITION_PREFIX_SCAN);
		ComputeShader* reallocatePositionShader = ShaderList::getInstance()->getComputeShader(RendEnum::REALLOCATE_RADIX_SORT);

		const unsigned numBits = 30;			// 10 bits per coordinate (3D)
		unsigned arraySize = unsigned(temperature.size());
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

		GLuint temperatureBuffer, indicesBufferID_1, indicesBufferID_2, pBitsBufferID, nBitsBufferID, positionBufferID;
		temperatureBuffer = ComputeShader::setReadBuffer(temperature, GL_STATIC_DRAW);
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

			bitMaskShader->bindBuffers(std::vector<GLuint> { temperatureBuffer, indicesBufferID_1, pBitsBufferID, nBitsBufferID });
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

		// Extract ordered values
		GLuint* sortedTemperatureData = ComputeShader::readData(indicesBufferID_2, GLuint());
		std::vector<GLuint> temperatureIndices = std::move(std::vector<GLuint>(sortedTemperatureData, sortedTemperatureData + temperature.size()));

		for (GLuint temperatureIdx: temperatureIndices)
		{
			sortedTemperature.push_back(temperature[temperatureIdx] / TEMPERATURE_MULTIPLICATOR);
		}

		// Delete buffers
		glDeleteBuffers(1, &indicesBufferID_1);
		glDeleteBuffers(1, &indicesBufferID_2);
		glDeleteBuffers(1, &pBitsBufferID);
		glDeleteBuffers(1, &nBitsBufferID);
		glDeleteBuffers(1, &positionBufferID);
		glDeleteBuffers(1, &temperatureBuffer);

		delete[] indices;
	}
}

void EnvPointCloud::threadedWritePointCloud(const std::string& filename, const bool ascii)
{
	std::filebuf fileBuffer;					
	fileBuffer.open(WRITE_POINT_CLOUD_FOLDER + filename, ascii ? std::ios::out : std::ios::out | std::ios::binary);

	std::ostream outstream(&fileBuffer);
	if (outstream.fail()) throw std::runtime_error("Failed to open " + filename + ".");

	tinyply::PlyFile pointCloud;

	std::vector<vec3> position;
	std::vector<vec3> rgb;
	std::vector<vec3> thermal;
	std::vector<float> temperature;

	for (int pointIdx = 0; pointIdx < _secondaryColor[PointColorTypes::THERMAL].size(); ++pointIdx)
	{
		if (_secondaryColor[PointColorTypes::THERMAL][pointIdx].w >= 1.0f)			// Otherwise it is not visible
		{
			position.push_back(_points[pointIdx]);
			rgb.push_back(_rgb[pointIdx]);
			thermal.push_back(_secondaryColor[PointColorTypes::THERMAL][pointIdx]);
			temperature.push_back(_secondaryColor[PointColorTypes::TEMPERATURE][pointIdx].y);
		}
	}

	const std::string componentName = "pointCloud";
	pointCloud.add_properties_to_element(componentName, { "x", "y", "z" }, tinyply::Type::FLOAT32, position.size(), reinterpret_cast<uint8_t*>(position.data()), tinyply::Type::INVALID, 0);
	pointCloud.add_properties_to_element(componentName, fromColorTypeToString(RGB_THERMAL), tinyply::Type::FLOAT32, rgb.size(), reinterpret_cast<uint8_t*>(rgb.data()), tinyply::Type::INVALID, 0);
	pointCloud.add_properties_to_element(componentName, fromColorTypeToString(THERMAL), tinyply::Type::FLOAT32, thermal.size(), reinterpret_cast<uint8_t*>(thermal.data()), tinyply::Type::INVALID, 0);
	pointCloud.add_properties_to_element(componentName, fromColorTypeToString(TEMPERATURE), tinyply::Type::FLOAT32, temperature.size(), reinterpret_cast<uint8_t*>(temperature.data()), tinyply::Type::INVALID, 0);
	pointCloud.get_comments().push_back("Research of GGGJ group at University of Jaen - Alfonso Lopez Ruiz");

	pointCloud.write(outstream, !ascii);
}

bool EnvPointCloud::writeToBinary(const std::string& filename)
{
	std::ofstream fout(filename, std::ios::out | std::ios::binary);
	if (!fout.is_open())
	{
		return false;
	}

	const size_t numPoints = _points.size();
	fout.write((char*)&numPoints, sizeof(size_t));
	fout.write((char*)&_points[0], numPoints * sizeof(vec4));
	fout.write((char*)&_rgb[0], numPoints * sizeof(vec3));
	fout.write((char*)&_aabb, sizeof(AABB));

	fout.close();

	return true;
}

/// [Static protected methods]

std::vector<std::string> EnvPointCloud::fromColorTypeToString(PointColorTypes colorType)
{
	switch (colorType)
	{
	case RGB:
		return std::vector<std::string> { "rgb_red", "rgb_green", "rgb_blue" };
		break;
	case RGB_THERMAL:
		return std::vector<std::string> { "rgbThermal_red", "rgbThermal_green", "rgbThermal_blue" };
		break;
	case THERMAL:
		return std::vector<std::string> { "thermal_red", "thermal_green", "thermal_blue" };
		break;
	case TEMPERATURE:
		return std::vector<std::string> { "temperature" };
		break;
	case UNUSED:
		return std::vector<std::string>();
		break;
	}

	return std::vector<std::string>();
}

/// [Static public methods]

void EnvPointCloud::indicateColorEntry(const bool defaultEntry, RenderingShader* shader, const PointColorTypes colorType)
{
	static std::vector<std::string> uniformImage{ "defaultColor", "secondaryColor_01", "secondaryColor_02", "secondaryColor_03", "secondaryColor_04" };

	if (defaultEntry) shader->setSubroutineUniform(GL_VERTEX_SHADER, "pointColorUniform", uniformImage[0]);
	else shader->setSubroutineUniform(GL_VERTEX_SHADER, "pointColorUniform", uniformImage[colorType + 1]);

	shader->applyActiveSubroutines();
}

void EnvPointCloud::indicatePointSize(RenderingShader* shader, RenderingParameters* rendParams, const EnvImage::ImageType imageType)
{
	shader->setUniform("pointSize", rendParams->_pointSize[imageType]);
}