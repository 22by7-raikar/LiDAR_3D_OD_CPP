//2D and 3D KD-Tree Implementations

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// Done: Inserting a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id); //root is a pointer, so &root is a pointer to pointer
	}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		// Tree is empty
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			// Calculate current dimension (0=x, 1=y)
			uint cd = depth % 2; //2D tree

			if(point[cd] < ((*node)->point[cd]))
			{
				insertHelper(&((*node)->left), depth + 1, point, id);
			}

			else
			{
				insertHelper(&((*node)->right), depth + 1, point, id);
			}
		}
	}

	void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if( (node->point[0] >= (target[0] - distanceTol)) && (node->point[0] <= (target[0] + distanceTol)) &&
			    (node->point[1] >= (target[1] - distanceTol)) && (node->point[1] <= (target[1] + distanceTol)) )
			{
				float distance = sqrt( (node->point[0] - target[0])*(node->point[0] - target[0]) +
				                       (node->point[1] - target[1])*(node->point[1] - target[1]) );

				if(distance <= distanceTol)
					ids.push_back(node->id);
			}

			// check across boundary
			uint cd = depth % 2;
			if( (target[cd] - distanceTol) < node->point[cd] )
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
				 
			if( (target[cd] + distanceTol) > node->point[cd] )
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};

// Structure to represent node of 3D kd tree
template<typename PointT> 
struct Node3D
{
	PointT point;
	int id;
	Node3D* left;
	Node3D* right;

	Node3D(PointT point, int setId)
	:	point(point), id(setId), left(NULL), right(NULL)
	{}

	~Node3D()
	{
		delete left;
		delete right;
	}
};

template<typename PointT>
struct KdTree3D
{
	Node3D<PointT>* root;

	KdTree3D()
	: root(NULL)
	{}

	~KdTree3D()
	{
		delete root;
	}

	void insert(PointT point, int id)
	{
		// Inserting a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id); //root is a pointer, so &root is a pointer to pointer
	}

	void insertHelper(Node3D<PointT>** node, uint depth, PointT point, int id)
	{
		// Tree is empty
		if(*node == NULL)
		{
			*node = new Node3D<PointT>(point, id);
		}
		else
		{
			// Calculate current dimension (0=x, 1=y, 2=z)
			uint cd = depth % 3; //3D tree
			float nodeVal, pointVal;
			if(cd == 0) {
				nodeVal = (*node)->point.x;
				pointVal = point.x;
			}
			else if(cd == 1) {
				nodeVal = (*node)->point.y;
				pointVal = point.y;
			}
			else {
				nodeVal = (*node)->point.z;
				pointVal = point.z;
			}

			if(pointVal < nodeVal)
			{
				insertHelper(&((*node)->left), depth + 1, point, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth + 1, point, id);
			}
		}
	}

	void searchHelper(PointT target, Node3D<PointT>* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			// Check if point is within bounding box
			if( (node->point.x >= (target.x - distanceTol)) && (node->point.x <= (target.x + distanceTol)) &&
			    (node->point.y >= (target.y - distanceTol)) && (node->point.y <= (target.y + distanceTol)) &&
			    (node->point.z >= (target.z - distanceTol)) && (node->point.z <= (target.z + distanceTol)) )
			{
				// Calculate actual Euclidean distance in 3D
				float distance = sqrt( (node->point.x - target.x)*(node->point.x - target.x) +
				                       (node->point.y - target.y)*(node->point.y - target.y) +
				                       (node->point.z - target.z)*(node->point.z - target.z) );

				if(distance <= distanceTol)
					ids.push_back(node->id);
			}

			// check across boundary
			uint cd = depth % 3;
			if(cd == 0)
			{
				if( (target.x - distanceTol) < node->point.x )
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				 
				if( (target.x + distanceTol) > node->point.x )
					searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
			else if(cd == 1)
			{
				if( (target.y - distanceTol) < node->point.y )
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				 
				if( (target.y + distanceTol) > node->point.y )
					searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
			else
			{
				if( (target.z - distanceTol) < node->point.z )
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				 
				if( (target.z + distanceTol) > node->point.z )
					searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};




