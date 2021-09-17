#pragma once

#include "stdafx.h"

//#include <pdal/Writer.hpp>
//#include <pdal/PointView.hpp>
//#include <pdal/PointTable.hpp>
//#include <pdal/Dimension.hpp>
//#include <pdal/Options.hpp>
//#include <pdal/StageFactory.hpp>
//#include <pdal/io/BufferReader.hpp>
#include "ImportedLibraries/tinyply.h"
#include "Graphics/Core/Group3D.h"

/**
*	@file FileManagement.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 08/07/2019
*/

/**
*	@brief Set of useful operations for file management.
*/
namespace FileManagement
{
	/**
	*	@brief
	*/
	void clearTokens(std::vector<std::string>& stringTokens, std::vector<float>& floatTokens);

	/**
	*	@brief Opens an imagen from a system file.
	*/
	bool openImage(const std::string& filename, std::vector<unsigned char>* image, unsigned int& width, unsigned int& height);

	/**
	*	@brief Loads the content of a binary file.
	*/
	bool readBinary(const std::string& filename, const std::vector<Model3D::ModelComponent*>& modelComp);

	/**
	*	@brief
	*/
	void readTokens(const std::string& line, const char delimiter, std::vector<std::string>& stringTokens, std::vector<float>& floatTokens);

	/**
	*	@brief Saves an image (array of bytes) in a system file, given the string filename.
	*/
	bool saveImage(const std::string& filename, std::vector<GLubyte>* image, const unsigned int width, const unsigned int height);
};

inline void FileManagement::clearTokens(std::vector<std::string>& stringTokens, std::vector<float>& floatTokens)
{
	stringTokens.clear();
	floatTokens.clear();
}

inline bool FileManagement::openImage(const std::string& filename, std::vector<unsigned char>* image, unsigned int& width, unsigned int& height)
{
	unsigned error = lodepng::decode(*image, width, height, filename.c_str());

	return error == 0;
}

inline bool FileManagement::readBinary(const std::string& filename, const std::vector<Model3D::ModelComponent*>& modelComp)
{
	std::ifstream fin (filename, std::ios::in | std::ios::binary);
	if (!fin.is_open())
	{
		return false;
	}

	size_t numVertices, numTriangles, numIndices;
	size_t it = 0;

	for (Model3D::ModelComponent* model: modelComp)
	{
		fin.read((char*)& numVertices, sizeof(size_t));
		model->_geometry.resize(numVertices);
		fin.read((char*)& model->_geometry[0], numVertices * sizeof(Model3D::VertexGPUData));

		fin.read((char*)& numTriangles, sizeof(size_t));
		model->_topology.resize(numTriangles);
		fin.read((char*)& model->_topology[0], numTriangles * sizeof(Model3D::FaceGPUData));

		fin.read((char*)& numIndices, sizeof(size_t));
		model->_triangleMesh.resize(numIndices);
		fin.read((char*)& model->_triangleMesh[0], numIndices * sizeof(GLuint));

		++it;
	}

	fin.close();

	return true;
}

inline void FileManagement::readTokens(const std::string& line, const char delimiter, std::vector<std::string>& stringTokens, std::vector<float>& floatTokens)
{
	std::stringstream ss(line);
	std::string tokenString;
	float tokenFloat;

	while (std::getline(ss, tokenString, delimiter))
	{
		if (!tokenString.empty())
		{
			try
			{
				tokenFloat = std::stof(tokenString);
				floatTokens.push_back(tokenFloat);
			}
			catch (std::exception& exception)
			{
				stringTokens.push_back(tokenString);
			}
		}
	}
}

inline bool FileManagement::saveImage(const std::string& filename, std::vector<GLubyte>* image, const unsigned int width, const unsigned int height)
{
	std::vector<unsigned char> result;
	unsigned error = lodepng::encode(result, *image, width, height);

	if (!error)
	{
		lodepng::save_file(result, filename);

		return true;
	}

	return false;
}