/**
 * 3D tree implementation: @author Uki D. Lucas UkiDLucas@gmail.com @UkiDLucas
 * 2D tree: @author Aaron Brown
 */
//#include "../../render/render.h"


/**
 * Structure representing a node of the KD tree
 */
struct Node
{
	std::vector<float> point;
	int pointCloudIndex; // unique identifier of the node
	Node* left; // child node with lesser value
	Node* right; // child node with bigger value

	/**
	 * Constructor for a new node
	 * @param point3D e.g. {-6.2, 7, 8} // meters
	 * @param setId
	 */
	Node(std::vector<float> point3D, int originalPointCloudIndex)
	:	point(point3D), pointCloudIndex(originalPointCloudIndex), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	// Node** node -- double pointer, memory address
	void insertHelper(Node** node, uint treeDepth, std::vector<float> point, int pointCloudIndex)
	{
		// dereference *node to see its actual value
		if(*node == NULL)
			// if you encouter the NULL node (root, or child), 
			// you assign this point in that place
			// dereference *node to set the actual value
			*node = new Node(point, pointCloudIndex);
		else
		{
			// For 2D pointers,
			// Depending on the treeDepth (every second, even|odd, 0|1 using mod 2)
			uint cd = treeDepth % 2; // results in 0 or 1
			if(point[cd] <= ((*node)->point[cd])) // it is possible that it is LESS OR EQUAL
				insertHelper(&((*node)->left), treeDepth+1, point, pointCloudIndex);
			else
				insertHelper(&((*node)->right), treeDepth+1, point, pointCloudIndex);
		}
		
	}

	void insert(std::vector<float> point, int pointCloudIndex)
	{
		uint treeDepth = 0;
		// insertHelper is a recursive function
		// passing in memory address of root node - which is a global pointer in struct KdTree
		insertHelper(&root, treeDepth, point, pointCloudIndex);
	}


	void searchHelper(
		std::vector<float> targetPoint, 
		Node* currentNode, 
		int treeDepth, 
		float distanceTreshhold, 
		std::vector<int>& resultIds) // address reference so we can keep changes
	{
		std::cout 
			<< std::endl 
			<< treeDepth
			<< " searchHelper: "
			<< " distanceTreshhold = " 
			<< distanceTreshhold 
			<< " meters, "
			<< "targetPoint = ";
		for (auto i: targetPoint)
  			std::cout << i << " "; 
		
		if(currentNode != NULL)
		{
			std::cout
				<< " current point = ";
			for (auto i: currentNode->point)
  				std::cout << i << " ";

			if( (  currentNode->point[0] >= (targetPoint[0] - distanceTreshhold) 
				&& currentNode->point[0] <= (targetPoint[0] + distanceTreshhold)) // Aaron has && here ?!?
				&&
				(  currentNode->point[1] >= (targetPoint[1] - distanceTreshhold)
				&& currentNode->point[1] <= (targetPoint[1] + distanceTreshhold)))
			{
				float distance = sqrt(
					( currentNode->point[0] - targetPoint[0]) * (currentNode->point[0] - targetPoint[0])
					+(currentNode->point[1] - targetPoint[1]) * (currentNode->point[1] - targetPoint[1]));
				std::cout << " distance = " << distance;
				if( distance <= distanceTreshhold)
				{
					std::cout << " adding current node pointCloudIndex to results <<<<<<<<<<<<<<<<< " << currentNode->pointCloudIndex ;
					resultIds.push_back(currentNode->pointCloudIndex); // add this pointCloudIndex to search results
				} 
			}

			// Mod % 2 to check if we are comparing X or Y coordinate
			// flow LEFT on the tree
			if( (targetPoint[treeDepth % 2] - distanceTreshhold) <= currentNode->point[treeDepth % 2] )
			{
				std::cout << " choosing LEFT branch " << std::endl;
				searchHelper(targetPoint, currentNode->left, treeDepth+1, distanceTreshhold, resultIds);
			}
			// else?
			// flow RIGHT on the tree
			if( (targetPoint[treeDepth % 2] + distanceTreshhold) > currentNode->point[treeDepth % 2] )			
			{
				std::cout << " choosing RIGHT branch " << std::endl;
				searchHelper(targetPoint, currentNode->right, treeDepth+1, distanceTreshhold, resultIds);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of targetPoint
	std::vector<int> search(std::vector<float> targetPoint, float distanceTreshhold)
	{
		std::vector<int> resultIds;
		int treeDepth = 0;
		searchHelper(targetPoint, root, treeDepth, distanceTreshhold, resultIds);
		
		std::cout << "search results: " << resultIds.size() << " ";
		
		for (auto i: resultIds)
  			std::cout << i << " ";
		std::cout << std::endl;

		return resultIds;
	}
};




