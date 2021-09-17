#pragma once

/**
*	@file OctreePointCloud.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 04/01/2021
*/

#include "DataStructures/RenderableOctree.h"
#include "Reconstruction/EnvPointCloud.h"

#define MAX_LEVEL INT_MAX
#define MAX_POINTS_NODE 10
#define NUM_CHILDREN 8

/**
*	@brief 
*/
class OctreePointCloud : public RenderableOctree
{
protected:
	class OctreeNode;								//!< Anidated class to be defined below

	// [Nodes references]
	OctreeNode*				_root;					//!< Node where the traversal starts

	// [Octree data]
	std::vector<vec4>*		_point;
	std::vector<vec3>*		_color;

	// [Octree metadata]
	uint8_t					_maxLevel;				//!< Higher priority than max triangles per node
	uint8_t					_maxPointsNode;			//!< Maximum capacity of a node

protected:
	/**
	*	@brief 
	*/
	void computeColor(OctreeNode* node, vec3& sumColor, unsigned& numColors);

	/**
	*	@brief Inserts a triangle at a certain node.
	*	@param node Node in which we're currently working.
	*	@param index Index of current point in our vector.
	*/
	void insert(OctreeNode* node, const unsigned index);

	/**
	*	@brief Finds the node where a point should be located. Warning: if the located node is null then its parent creates its children.
	*	@param node Node where we're currently working.
	*	@param parent Previous node.
	*	@param index Index of current point in our vector.
	*/
	OctreeNode* locateNode(OctreeNode* node, OctreeNode* parent, const unsigned index);

	/**
	*	@brief Obtains the metadata of each node in a recursive way (bounding box of each node).
	*	@param node Node where we're searching.
	*	@param aabb Retrieved aligned axis bounding boxes.
	*/
	void retrieveNodeData(OctreeNode* node, std::vector<AABB>& aabb, const unsigned maxLevel);

	/**
	*	@brief 
	*/
	void retrieveSimplifiedPointCloud(OctreeNode* node, const unsigned maxLevel, std::vector<vec4>& point, std::vector<vec3>& color);

public:
	/**
	*	@brief Constructor of an octree from a point cloud and its colors.
	*/
	OctreePointCloud(EnvPointCloud* pointCloud);

	/**
	*	@brief Unsupported copy constructor.
	*/
	OctreePointCloud(const OctreePointCloud& orig) = delete;

	/**
	*	@brief Destructor.
	*/
	virtual ~OctreePointCloud();

	/**
	*	@return AABB which marks the octree boundaries.
	*/
	AABB getAABB() const { return _root->_aabb; }

	/**
	*	@brief Returns the bounding boxes for each node of the octree for rendering purposes.
	*	@param aabb Vector of axis-aligned bounding boxes which represent the octree.
	*/
	virtual void getAABBs(std::vector<AABB>& aabb, const unsigned maxLevel = UINT_MAX);

	/**
	*	@brief Retrieves a point cloud where those nodes with depth > maxLevel are simplified into a point with an averaged color.
	*/
	virtual void retrieveSimplifiedPointCloud(const unsigned maxLevel, std::vector<vec4>& point, std::vector<vec3>& color);

	/**
	*	@brief Unsupported assignment overriding.
	*/
	OctreePointCloud& operator=(const OctreePointCloud& orig) = delete;

	// Setters
	
	/**
	*	@brief
	*/
	void setMaximumLevel(const unsigned maxLevel) { _maxLevel = maxLevel; }
	
	/**
	*	@brief
	*/
	void setMaximumPointsNode(const unsigned maxPointsNode) { _maxPointsNode = maxPointsNode; }

protected:
	/**
	*	@brief 
	*/
	class OctreeNode
	{
	public:
		AABB								_aabb;
		std::vector<OctreeNode*>			_children;						//!< Nodes which are behind this one
		uint8_t								_depth;							//!< Node depth
		OctreePointCloud*					_octree;
		std::list<unsigned>					_pointIndex;					//!< 

	public:
		/**
		*	@brief Constructor.
		*	@param depth Level where this node is located at the tree.
		*	@param aabb Bounding box that represents the boundaries of this node.
		*/
		OctreeNode(uint8_t depth, const AABB& aabb, OctreePointCloud* octree);

		/**
		*	@brief Destructor.
		*/
		~OctreeNode();

		/**
		*	@brief Initializes all the children pointers.
		*/
		void createChildren();

		/**
		*	@brief Searchs the octree node where the point should be located.
		*/
		int getChildrenIndex(vec3& point);

		/**
		*	@brief Search the octree node in which the point (located at 'index') could be inserted.
		*/
		uint8_t getChildrenIndices(const unsigned index);

		/**
		*	@return Amount of points the childs contains.
		*/
		size_t getChildrenSize();

		/**
		*	@return True if the node is a leaf. 
		*/
		bool isLeaf() { return !_children.size(); }
	};
};

