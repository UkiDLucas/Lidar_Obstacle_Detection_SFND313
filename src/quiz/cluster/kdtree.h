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
	void insertHelper(Node** node, uint treeDepth, std::vector<float> point, int id)
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
			// Depending on the treeDepth (every second, even|odd, 0|1 using mod 2)
			uint cd = treeDepth % 2; // results in 0 or 1
			if(point[cd] <= ((*node)->point[cd])) // it is possible that it is LESS OR EQUAL
				insertHelper(&((*node)->left), treeDepth+1, point, id);
			else
				insertHelper(&((*node)->right), treeDepth+1, point, id);
		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		uint treeDepth = 0;
		// insertHelper is a recursive function
		// passing in memory address of root node - which is a global pointer in struct KdTree
		insertHelper(&root, treeDepth, point, id);
	}


	void searchHelper(
		std::vector<float> targetPoint, 
		Node* currentNode, 
		int treeDepth, 
		float distanceTreshhold, 
		std::vector<int> resultIds)
	{
		if(currentNode != NULL)
		{
			if   ( currentNode->point[0] >= (targetPoint[0] - distanceTreshhold) // check if point is withing BOX. The 0 is X coordinate
				&& currentNode->point[0] <= (targetPoint[0] + distanceTreshhold)
				&& currentNode->point[1] >= (targetPoint[1] - distanceTreshhold) // The 1 is Y coordinate
				&& currentNode->point[1] <= (targetPoint[1] + distanceTreshhold))
			{
				float distance = sqrt(
					( currentNode->point[0] * targetPoint[0]) * (currentNode->point[0] * targetPoint[0])
					+(currentNode->point[1] * targetPoint[1]) * (currentNode->point[1] * targetPoint[1]));
				if( distance <= distanceTreshhold)
					resultIds.push_back(currentNode->id); // add this id to search results
			}

			//check accross bondries
			if( (targetPoint[treeDepth % 2] - distanceTreshhold) <= currentNode->point[treeDepth % 2] )
				searchHelper(targetPoint, currentNode->left, treeDepth+1, distanceTreshhold, resultIds);

			
			if( (targetPoint[treeDepth % 2] + distanceTreshhold) > currentNode->point[treeDepth % 2] )
				searchHelper(targetPoint, currentNode->right, treeDepth+1, distanceTreshhold, resultIds);
		}
	}




	// return a list of point ids in the tree that are within distance of targetPoint
	std::vector<int> search(std::vector<float> targetPoint, float distanceTreshhold)
	{
		std::vector<int> ids;
		int treeDepth = 0;
		searchHelper(targetPoint, root, treeDepth, distanceTreshhold, ids);
		return ids;
	}
	

};




