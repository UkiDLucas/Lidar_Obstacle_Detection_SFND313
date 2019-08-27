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
     * @param point - this is a point we try to assign. Example point3D {22.9123, 7.071, 1.02}
     * @param index - this is the INDEX of the ORIGINAL Point Cloud
     */
    void insertPointToCorrectNode(Node **node, uint treeDepth, std::vector<float> point, int index) {

//        cout << "insertPointToCorrectNode() point at " << point.at(0) << ", " << point.at(1) << ", "  << point.at(2) << endl;
//        cout << "insertPointToCorrectNode() point []" << point[0] << ", " << point[1] << ", "  << point[2] << endl;

        if (*node == NULL) {
            // if you encounter the NULL (root, or child) node,
            // assign given point in that node.
            // dereference *node to get or set the actual value
            *node = new Node(point, index);
            return; // nothing else to do
        }

        // Every vertical layer (treeDepth) of the tree
        // is dedicated to one coordinate comparison only, e.g. x, y, or z.
        // We are using mod 3 to achieve that.
        uint xyz = treeDepth % 3; // results in 0, 1, or 2

//            cout << "insertPointToCorrectNode() "  << endl;
//            cout << "insertPointToCorrectNode() treeDepth " << treeDepth << endl;
//            cout << "insertPointToCorrectNode() xyz " << xyz << endl;
//            cout << "insertPointToCorrectNode() (*node)->point[xyz] " << (*node)->point[xyz] << endl;
//            cout << "insertPointToCorrectNode() point[xyz] " << point[xyz] << endl;

        // The following variables are superficial, but greatly add to readability and maintenance of the code.
        // TODO in production I might remove the variables to speed up the code.
        float incomingValue = point[xyz]; // from the Point Cloud we insert
        float existingValue = (*node)->point[xyz]; // of the existing node

        if (incomingValue <= existingValue)
        {
            // Incoming lesser or equal values go LEFT
            insertPointToCorrectNode(&((*node)->leftNode), treeDepth + 1, point, index);
        }
        else {
            // Incoming greater values go RIGHT
            insertPointToCorrectNode(&((*node)->rightNode), treeDepth + 1, point, index);
        }
    }


private:
    /**
     * You need 3D (including Z) detection, especially for bridges, trees, street wires, etc.
     * @param pointA - incoming point we are trying to evaluate
     * @param pointB - the node in the KD Tree we are comparing to
     * @param threshold - the additional distance beyond node that are acceptable
     * @return
     */
    bool isNearby(
            std::vector<float> pointA,
            std::vector<float> pointB,
            float threshold
            )
    {
        // QUICK CHECK pointB is within target threshold bounding box
        // Since all conditions are AND (&&) then no parenthesis grouping is needed.
        if (pointA[0] >= pointB[0] - threshold
            && pointA[0] <= pointB[0] + threshold
            && pointA[1] >= pointB[1] - threshold
            && pointA[1] <= pointB[1] + threshold
            && pointA[2] >= pointB[2] - threshold
            && pointA[2] <= pointB[2] + threshold
           )
        {
            //std::cout << " distance IS WITHIN the TRESHOLD! Quick Check " << std::endl;

            // DETAILED CHECK for distance of circum-sphere
            // d = √ [(x2-x1)2 + (y2-y1)2 + (z2-z1)2]
            float distance = sqrt(
                    (pointB[0] - pointA[0]) * (pointB[0] - pointA[0])
                    + (pointB[1] - pointA[1]) * (pointB[1] - pointA[1])
                    + (pointB[2] - pointA[2]) * (pointB[2] - pointA[2])
            );

            if (distance <= threshold) {
                //std::cout << "isNearby() distance = " << distance << std::endl;
                return true;
            }
        }
        return false;
    }






private:
    void searchDistanceNodeToPoint(
            vector < float > incomingPoint,
            Node *currentNode,
            int treeDepth,
            float distanceTreshold,
            vector< vector < float > > &nearPoints) // address reference so we can keep changes
    {
        if (currentNode == NULL)
        {
            std::cerr << " Came to a NULL NODE, point was not close to any nodes of the KD Tree." << std::endl;
            // If the current node node does not exist, there is nothing to search
            return;
        }

        // PRINT
        std::cout
                << std::endl
                << "depth: " << treeDepth
//                << " treshold = "
//                << distanceTreshold
//                << " meters, "
                << std::endl;
//                << " incoming point     = ";
//        for (auto i: incomingPoint)
//            std::cout << i << " ";
//        cout << std::endl;
//        std::cout
//                << " current node point = ";
//        for (auto i: currentNode->point)
//            std::cout << i << " ";
//        cout << std::endl;

        vector < float > point = currentNode->point;

        if (isNearby(incomingPoint, point, distanceTreshold))
        {
            cout << " isNearby(), adding current node to results!" << endl;
            nearPoints.push_back(point); // add this pointCloudIndex to search results
            return;
        }

        // keep looking in deeper nodes
        // Every vertical layer (treeDepth) of the tree
        // is dedicated to one coordinate comparison only, e.g. x, y, or z.
        // We are using mod 3 to achieve that.
        uint xyz = treeDepth % 3; // results in 0, 1, or 2

        // The following variables are superficial, but greatly add to readability and maintenance of the code.
        float incomingValue = incomingPoint[xyz];
        float currentValue = currentNode->point[xyz];

        // IMPORTANT: this needs to be the EXACTLY SAME LOGIC as TREE INSERT.
        if ( incomingValue <= currentValue ) { // flow LEFT on the tree
            std::cout << " search LEFT branch " << std::endl;
            searchDistanceNodeToPoint(
                    incomingPoint,
                    currentNode->leftNode,
                    treeDepth + 1,
                    distanceTreshold,
                    nearPoints);
        } else
        {
            std::cout << " search RIGHT branch " << std::endl;
            searchDistanceNodeToPoint(
                    incomingPoint,
                    currentNode->leftNode,
                    treeDepth + 1,
                    distanceTreshold,
                    nearPoints);
        }

    }








public:
    /**
     *
     * @param point - vector of floats X, Y, Z
     * @param index - int representing the INDEX of the original PointCloud
     */
    void insert(std::vector<float> point, int index)
    {
        uint treeDepth = 0;
        // insertPointToCorrectNode is a recursive function
        // passing in memory address of root node - which is a global pointer in struct KdTree3D
        insertPointToCorrectNode(&root, treeDepth, point, index);
    }



public:
	// return a list of point IDs in the tree that are within distance of the targetPoint
    vector < vector < float > >
            search(vector< float > incomingPoint, float distanceTreshhold)
	{
		vector < vector < float > > nearPoints;
		int treeDepth = 0;
        searchDistanceNodeToPoint(incomingPoint, root, treeDepth, distanceTreshhold, nearPoints);
		
//		std::cout << "search() results: " << resultIds.size() << std::endl;
		
//		for (auto i: resultIds)
//  			std::cout << i << " ";
//		std::cout << std::endl;

		return nearPoints;
	}





    public:
	/**
	 * Constructor is setting the root NULL
	 */
    KdTree3D()
            : root(NULL)
    {}
};




