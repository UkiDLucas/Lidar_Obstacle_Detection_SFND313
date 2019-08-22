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

using namespace std;
using namespace pcl;

/**
 * \brief Provide a pointer to the input dataset.
 * \param[in] cloud the const boost shared pointer to a PointCloud message
 * \param[in] indices the point indices subset that is to be used from \a cloud
 **/
//void setInputCloud (const pcl::PCLPointCloud2ConstPtr cloud, const pcl::IndicesConstPtr indices);


/**
 * Structure representing a node of the KD tree
 */
struct Node
{
    public:
    int pointCloudIndex; // unique identifier of the node
    Node* leftNode; // child node with lesser value
    Node* rightNode; // child node with bigger value
    std::vector<float> point;

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
private:
    Node *root;



private:
    /**
     * ASSIGN GIVEN POINT TO THE CORRECT NODE
     * @param node - Node** node -- double pointer, memory address. This is the current node we operate on.
     * @param treeDepth This is the horizontal layer of the tree we are on, counting from the top.
     * @param point3D - this is a point we try to assign. Example point3D {22.9123, 7.071, 1.02}
     * @param index - this is the INDEX of the ORIGINAL Point Cloud
     */
    void assignPointToCorrectNode(Node **node, uint treeDepth, std::vector<float> point3D, int index) {

//        cout << "assignPointToCorrectNode() point3D at " << point3D.at(0) << ", " << point3D.at(1) << ", "  << point3D.at(2) << endl;
//        cout << "assignPointToCorrectNode() point3D []" << point3D[0] << ", " << point3D[1] << ", "  << point3D[2] << endl;

        if (*node == NULL)
            // if you encounter the NULL (root, or child) node,
            // assign given point3D in that node.
            // dereference *node to set the actual value
            *node = new Node(point3D, index);
        else {
            // Every vertical layer (treeDepth) of the tree
            // is dedicated to one coordinate comparison only, e.g. x, y, or z.
            // We are using mod 3 to achieve that.
            uint xyz = treeDepth % 3; // results in 0, 1, or 2

//            cout << "assignPointToCorrectNode() "  << endl;
//            cout << "assignPointToCorrectNode() treeDepth " << treeDepth << endl;
//            cout << "assignPointToCorrectNode() xyz " << xyz << endl;
//            cout << "assignPointToCorrectNode() (*node)->point[xyz] " << (*node)->point[xyz] << endl;
//            cout << "assignPointToCorrectNode() point3D[xyz] " << point3D[xyz] << endl;

            // The following variables are superficial, but greatly add to readability and maintenance of the code.
            // TODO in production I might remove the variables to speed up the code.
            float incomingValue = point3D[xyz]; // from the Point Cloud we insert
            float existingValue = (*node)->point[xyz]; // of the existing node

            if (incomingValue <= existingValue)
            {
                // Incoming lesser or equal values go LEFT
                assignPointToCorrectNode(&((*node)->leftNode), treeDepth + 1, point3D, index);
            }
            else {
                // Incoming greater values go RIGHT
                assignPointToCorrectNode(&((*node)->rightNode), treeDepth + 1, point3D, index);
            }
        }
    }


private:
    void searchDistanceNodeToPoint(
            std::vector<float> targetPoint,
            Node *currentNode,
            int treeDepth,
            float distanceTreshold,
            std::vector<int> &resultIds) // address reference so we can keep changes
    {
        std::cout
                << std::endl
                << "depth: " << treeDepth
                << " treshold = "
                << distanceTreshold
                << " meters, " << std::endl
                << "targetPoint = ";
        for (auto i: targetPoint)
            std::cout << i << " ";
        cout << std::endl;

        // If the current node node does not exist, there is nothing to search
        if (currentNode != NULL) {
            std::cout << " current node point = ";

            for (auto i: currentNode->point)
                std::cout << i << " ";

            cout << std::endl;
            //  You need 3D (including Z) detection, especially for bridges, trees, street wires, etc.

            // The following variables are superficial, but greatly add to readability and maintenance of the code.
            // TODO in production I might remove the variables to speed up the code.
            float nodeX = currentNode->point[0];
            float nodeY = currentNode->point[1];
            float nodeZ = currentNode->point[2];
            float targetX = targetPoint[0];
            float targetY = targetPoint[1];
            float targetZ = targetPoint[2];
            float boundryX = targetX - distanceTreshold;
            float boundryY = targetY - distanceTreshold;
            float boundryZ = targetZ - distanceTreshold;

            bool isNodeFoundForThisPoint = false;


            float distance = sqrt(
                    (nodeX - targetX) * (nodeX - targetX)
                    + (nodeY - targetY) * (nodeY - targetY)
                    + (nodeZ - targetZ) * (nodeZ - targetZ)
            );
            std::cout << " distance = " << distance << std::endl;



            // QUICK CHECK node is within target threshold bounding box
            if ((nodeX >= boundryX && nodeX <= boundryX)
                &&
                (nodeY >= boundryY && nodeY <= boundryY)
                &&
                (nodeZ >= boundryZ && nodeZ <= boundryZ))
            {
                // DETAILED CHECK for distance of circum-sphere
                // d = √ [(x2-x1)2 + (y2-y1)2 + (z2-z1)2]
                float distance = sqrt(
                        (nodeX - targetX) * (nodeX - targetX)
                        + (nodeY - targetY) * (nodeY - targetY)
                        + (nodeZ - targetZ) * (nodeZ - targetZ)
                        );
                std::cout << " distance = " << distance << std::endl;

                if (distance <= distanceTreshold) {
                    std::cout << " adding current node pointCloudIndex to results <<<<<<<<<<<<<<<<< "
                              << currentNode->pointCloudIndex;

                    resultIds.push_back(currentNode->pointCloudIndex); // add this pointCloudIndex to search results
                    isNodeFoundForThisPoint = true;
                }
            }else{


            }


            if(!isNodeFoundForThisPoint) // keep looking in deeper nodes
            {
                // Every vertical layer (treeDepth) of the tree
                // is dedicated to one coordinate comparison only, e.g. x, y, or z.
                // We are using mod 3 to achieve that.
                uint xyz = treeDepth % 3; // results in 0, 1, or 2

                // The following variables are superficial, but greatly add to readability and maintenance of the code.
                // TODO in production I might remove the variables to speed up the code.

                float incomingValue = targetPoint[xyz];
                float currentValue = currentNode->point[xyz];

                // IMPORTANT: this needs to be the EXACTLY SAME LOGIC as TREE INSERT.
                if ( incomingValue <= currentValue ) { // flow LEFT on the tree
                    std::cout << " search LEFT branch " << std::endl;
                    searchDistanceNodeToPoint(targetPoint, currentNode->leftNode, treeDepth + 1, distanceTreshold,
                                              resultIds);
                } else{
                    std::cout << " search RIGHT branch " << std::endl;
                    searchDistanceNodeToPoint(targetPoint, currentNode->rightNode, treeDepth + 1, distanceTreshold,
                                              resultIds);
                }
            }
        } // if (currentNode != NULL)
    }








    public:
    void insert(std::vector<float> point, int pointCloudIndex)
    {
        uint treeDepth = 0;
        // assignPointToCorrectNode is a recursive function
        // passing in memory address of root node - which is a global pointer in struct KdTree3D
        assignPointToCorrectNode(&root, treeDepth, point, pointCloudIndex);
    }



    public:
	// return a list of point IDs in the tree that are within distance of the targetPoint
	std::vector<int> search(std::vector<float> targetPoint, float distanceTreshhold)
	{
		std::vector<int> resultIds;
		int treeDepth = 0;
        searchDistanceNodeToPoint(targetPoint, root, treeDepth, distanceTreshhold, resultIds);
		
//		std::cout << "search() results: " << resultIds.size() << std::endl;
		
//		for (auto i: resultIds)
//  			std::cout << i << " ";
//		std::cout << std::endl;

		return resultIds;
	}





    public:
	/**
	 * Constructor is setting the root NULL
	 */
    KdTree3D()
            : root(NULL)
    {}
};




