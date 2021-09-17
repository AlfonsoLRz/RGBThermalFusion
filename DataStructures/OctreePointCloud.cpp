#include "stdafx.h"
#include "OctreePointCloud.h"

#include "Geometry/3D/Intersections3D.h"

/// [Public methods]

OctreePointCloud::OctreePointCloud(EnvPointCloud* pointCloud)
	: _maxLevel(UINT8_MAX), _maxPointsNode(MAX_POINTS_NODE), _root(nullptr), _point(nullptr), _color(nullptr)
{
	if (pointCloud)
	{
		AABB aabb = pointCloud->getAABB();
		aabb.update(aabb.max() + glm::epsilon<float>() * 100.0f);
		aabb.update(aabb.min() - glm::epsilon<float>() * 100.0f);

		_point = pointCloud->getPoints();
		_color = pointCloud->getColors();
		_root = new OctreeNode(0, aabb, this);

		// Push back function
		{
			const unsigned numPoints = pointCloud->getNumberOfPoints();

			for (unsigned point = 0; point < numPoints; ++point)
			{
				OctreeNode* node = locateNode(_root, nullptr, point);
				insert(node, point);
			}
		}
	}
}

OctreePointCloud::~OctreePointCloud()
{
	delete _root;
}

void OctreePointCloud::getAABBs(std::vector<AABB>& aabb, const unsigned maxLevel)
{
	this->retrieveNodeData(_root, aabb, maxLevel);
}

void OctreePointCloud::retrieveSimplifiedPointCloud(const unsigned maxLevel, std::vector<vec4>& point, std::vector<vec3>& color)
{
	this->retrieveSimplifiedPointCloud(_root, maxLevel, point, color);
}

/// [Protected methods]

void OctreePointCloud::computeColor(OctreeNode* node, vec3& sumColor, unsigned& numColors)
{
	if (node->isLeaf())
	{
		for (unsigned& pointIndex : node->_pointIndex)
		{
			sumColor += _color->at(pointIndex);
			++numColors;
		}

		return;
	}

	for (int children = 0; children < node->_children.size(); ++children)
	{
		this->computeColor(node->_children[children], sumColor, numColors);
	}
}

void OctreePointCloud::insert(OctreeNode* node, const unsigned index)
{
	node->_pointIndex.push_back(index);

	if (node->_depth != _maxLevel && node->_pointIndex.size() > _maxPointsNode)
	{
		node->createChildren();

		for (unsigned& pointIndex: node->_pointIndex)
		{
			uint8_t childrenIndex = node->getChildrenIndices(pointIndex);

			if (childrenIndex != UINT8_MAX)
			{
				insert(node->_children[childrenIndex], pointIndex);
			}
		}

		node->_pointIndex.clear();
	}
}

OctreePointCloud::OctreeNode* OctreePointCloud::locateNode(OctreeNode* node, OctreeNode* parent, const unsigned index)
{
	if (node->isLeaf())
	{
		return node;
	}

	uint8_t childrenIndex = node->getChildrenIndices(index);
	return locateNode(node->_children[childrenIndex], node, index);
}

void OctreePointCloud::retrieveNodeData(OctreeNode* node, std::vector<AABB>& aabb, const unsigned maxLevel)
{
	if (node->isLeaf() || node->_depth == maxLevel)
	{
		aabb.push_back(node->_aabb);

		return;
	}

	for (int children = 0; children < node->_children.size(); ++children)
	{
		this->retrieveNodeData(node->_children[children], aabb, maxLevel);
	}
}

void OctreePointCloud::retrieveSimplifiedPointCloud(OctreeNode* node, const unsigned maxLevel, std::vector<vec4>& point, std::vector<vec3>& color)
{
	if (node->isLeaf())
	{
		for (unsigned& index : node->_pointIndex)
		{
			point.push_back(_point->at(index));
			color.push_back(_color->at(index));
		}

		return;
	}

	if (node->_depth == maxLevel)
	{
		point.push_back(vec4(node->_aabb.center(), 1.0f));

		// Compute color from all those points stored behind this node
		vec3 sumColor(.0f);
		unsigned numColors = 0;

		this->computeColor(node, sumColor, numColors);
		color.push_back(vec3(sumColor.x / numColors, sumColor.y / numColors, sumColor.z / numColors));
	}
	else
	{
		for (int children = 0; children < node->_children.size(); ++children)
		{
			this->retrieveSimplifiedPointCloud(node->_children[children], maxLevel, point, color);
		}
	}
}

/// [Node subclass]

OctreePointCloud::OctreeNode::OctreeNode(uint8_t level, const AABB& aabb, OctreePointCloud* octree) :
	_aabb(aabb), _depth(level), _octree(octree)
{
}

OctreePointCloud::OctreeNode::~OctreeNode()
{
	for (int i = 0; i < _children.size(); ++i)
	{
		delete _children[i];
		_children.clear();
	}
}

void OctreePointCloud::OctreeNode::createChildren()
{
	if (isLeaf())
	{
		std::vector<AABB> childrenAABB = _aabb.split(2);

		for (int i = 0; i < NUM_CHILDREN; ++i)
		{
			_children.push_back(new OctreeNode(_depth + 1, childrenAABB[i], _octree));
		}
	}
}

int OctreePointCloud::OctreeNode::getChildrenIndex(vec3& point)
{
	vec3 center = _aabb.center();
	int childrenIndex = 0;

	if (point.y > center.y) childrenIndex += 4;				// X and Z are first explored
	if (point.z > center.z) childrenIndex += 2;				// Z is explored in second place
	if (point.x > center.x) childrenIndex += 1;

	return childrenIndex;
}

uint8_t OctreePointCloud::OctreeNode::getChildrenIndices(const unsigned index)
{
	for (int i = 0; i < _children.size(); ++i)
	{
		if (Intersections3D::inside(_children[i]->_aabb, _octree->_point->at(index)))
		{
			return i;
		}
	}

	return UINT8_MAX;
}

size_t OctreePointCloud::OctreeNode::getChildrenSize()
{
	size_t size = 0;

	for (int i = 0; i < _children.size(); ++i)
	{
		if (_children[i])
		{
			size += _children[i]->_pointIndex.size();
		}
	}

	return size;
}