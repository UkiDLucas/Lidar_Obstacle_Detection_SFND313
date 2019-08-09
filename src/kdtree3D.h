/*!
 *  \brief     KD Tree
 *  \details   The KD Tree is a data structure that organizes the Point Cloud Data by proximity, therefore, it allows for a very fast look-up times.
 *  \author    Aaron Brown https://www.linkedin.com/in/awbrown90/
 *  \author    Uki D. Lucas https://www.linkedin.com/in/ukidlucas/
 *  \date      August 1, 2019
 *  \bug       It is still 2D implementation
 *  \warning   It is still 2D implementation
 *  \copyright code_reuse_license.md
 */

//#include "../../render/render.h"


/** \brief Provide a pointer to the input dataset.
  * \param[in] cloud the const boost shared pointer to a PointCloud message
  * \param[in] indices the point indices subset that is to be used from \a cloud
  */
void
setInputCloud (const pcl::PCLPointCloud2ConstPtr cloud,
               const pcl::IndicesConstPtr indices = IndicesConstPtr ());


/**
 * Structure representing a node of the KD tree
 */
struct Node
{
	std::vector<float> point;
	int pointCloudIndex; // unique identifier of the node
	Node* leftNode; // child node with lesser value
	Node* rightNode; // child node with bigger value

	/**
	 * Constructor for a new node
	 * @param point3D e.g. {-6.2, 7, 8} // meters
	 * @param setId
	 */
	Node(std::vector<float> point3D, int originalPointCloudIndex)
	:	point(point3D), pointCloudIndex(originalPointCloudIndex), leftNode(NULL), rightNode(NULL)
	{}
};

struct KdTree3D
{
	Node* root;

    KdTree3D()
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
				insertHelper(&((*node)->leftNode), treeDepth+1, point, pointCloudIndex);
			else
				insertHelper(&((*node)->rightNode), treeDepth+1, point, pointCloudIndex);
		}
		
	}

    setInputCloud

	void insert(std::vector<float> point, int pointCloudIndex)
	{
		uint treeDepth = 0;
		// insertHelper is a recursive function
		// passing in memory address of root node - which is a global pointer in struct KdTree3D
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

            // TODO most of the time X and Y point are enough to establish proximity,
            //  however you can run into problems under the bridges, trees, street wires, etc.

            float nodeX = currentNode->point[0];
            float nodeY = currentNode->point[1];
            float targetX = targetPoint[0];
            float targetY = targetPoint[1];

            // QUICK CHECK node is within target treshhold bounding box
			if((   nodeX >= (targetX - distanceTreshhold) && nodeX <= (targetX + distanceTreshhold))
				&&
               (   nodeY >= (targetY - distanceTreshhold) && nodeY <= (targetY + distanceTreshhold)))
			{
			    // DETAILED CHECK for distance of circum-sphere
				float distance = sqrt(
                        (nodeX - targetX) * (nodeX - targetX)
					+ (nodeY - targetY) * (nodeY - targetY));
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
				searchHelper(targetPoint, currentNode->leftNode, treeDepth+1, distanceTreshhold, resultIds);
			}
			// else?
			// flow RIGHT on the tree
			if( (targetPoint[treeDepth % 2] + distanceTreshhold) > currentNode->point[treeDepth % 2] )			
			{
				std::cout << " choosing RIGHT branch " << std::endl;
				searchHelper(targetPoint, currentNode->rightNode, treeDepth+1, distanceTreshhold, resultIds);
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




