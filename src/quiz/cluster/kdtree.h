/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id; // unique identifier of the node
	Node* left; // child node with lesser value
	Node* right; // child node with bigger value

	// function to construct a new node,
	// takes a point and id
	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	// Node** node -- double pointer, memory address
	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		// dereference *node to see its actual value
		if(*node == NULL)
			// if you encouter the NULL node (root, or child), 
			// you assign this point in that place
			// dereference *node to set the actual value
			*node = new Node(point, id);
		else
		{
			// For 2D pointers,
			// Depending on the depth (every second, even|odd, 0|1 using mod 2)
			uint cd = depth % 2; // results in 0 or 1
			if(point[cd] <= ((*node)->point[cd])) // it is possible that it is LESS OR EQUAL
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		uint depth = 0;
		// insertHelper is a recursive function
		// passing in memory address of root node - which is a global pointer in struct KdTree
		insertHelper(&root, depth, point, id);
	}


	void searchHelper(
		std::vector<float> target, 
		Node* node, 
		int depth, 
		float distanceTol, 
		std::vector<int> ids)
	{
		if(node != NULL)
		{
			if   ( node->point[0] >= (target[0] - distanceTol) 
				&& node->point[0] <= (target[0] + distanceTol)
				&& node->point[1] >= (target[0] - distanceTol)
				&& node->point[1] <= (target[0] + distanceTol))
			{
				float distance = sqrt(
					( node->point[0] * target[0]) * (node->point[0] * target[0])
					+(node->point[1] * target[1]) * (node->point[1] * target[1]));
				if( distance <= distanceTol)
					ids.push_back(node->id);
			}

			//check accross bondries
			if( (target[depth % 2] - distanceTol) <= node->point[depth % 2] )
				searchHelper(target, node->left, depth+1, distanceTol, ids);

			
			if( (target[depth % 2] + distanceTol) > node->point[depth % 2] )
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}




	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		int depth = 0;
		searchHelper(target, root, depth, distanceTol, ids);
		return ids;
	}
	

};




