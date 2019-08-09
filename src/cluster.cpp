/*!
 *  \brief     Euclidean Distance Clustering
 *  \details   https://en.wikipedia.org/wiki/Euclidean_distance
 *  \author    Aaron Brown https://www.linkedin.com/in/awbrown90/
 *  \author    Uki D. Lucas https://www.linkedin.com/in/ukidlucas/
 *  \date      August 1, 2019
 *  \bug       TBD
 *  \warning   TBD
 *  \copyright code_reuse_license.md
 */

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h" // 2D


/**
 * returns the list of cluster indices
 * To perform the clustering, 
 * iterate through each point in the cloud,
 * keep track of which points have been processed already.
 * For each point add it to a list of points defined as a cluster, 
 * then get a list of all the points in close proximity to that point by using the search function.
 * For each point in close proximity that hasn't already been processed, 
 * add it to the cluster and repeat the process of calling proximity points.
 * Once the recursion stops for the first cluster, 
 * create a new cluster and move through the point list, repeating the above process for the new cluster. 
 * Once all the points have been processed, 
 * there will be a certain number of clusters found, 
 * return as a list of clusters.
 */
std::vector<std::vector<int>> 
findPointCloudClusters(
	const std::vector<std::vector<float>>& points, 
	KdTree* tree, 
	float distanceThreshold)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false); // same amount as incoming points, all default false.

	int i = 0;
	while(i < points.size())
	{
		if(processed[i]) // Was this point was processed?
		{
			i++; // move to the next point index
			continue; 
		}
		// This point was NOT processed.
		// Create a new cluster.
		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceThreshold);
		clusters.push_back(cluster);
		i++; // move to the next point index
	}
	return clusters;
}

int main ()
{
	float distanceThreshold = 3.0; // meters
	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min =   0;
  	window.z_max =   0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = { 
		{-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	KdTree* tree = new KdTree;
  
    for (int i=0; i < points.size(); i++)
	{
		// insert points into the tree
    	tree->insert(points[i], i); // actual point and original index
	}

  	int it = 0;
  	render2DTree(tree->root, viewer, window, it);
  
  	//std::cout << "Start TEST SEARCH cluster.cpp" << std::endl;
	//std::vector<float> targetPoint = {-6, 7};
  	//std::vector<int> nearbyPointIds = tree->search(targetPoint, distanceThreshold);
	//std::cout << " nearby point IDs: ";
  	//for(int index : nearbyPointIds)
    //  std::cout << index << ", ";
  	//std::cout << std::endl;
	//std::cout << "End TEST SEARCH cluster.cpp" << std::endl;


  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	
  	std::vector<std::vector<int>> clusters = findPointCloudClusters(points, tree, distanceThreshold);

  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout 
	  << "For point cloud of  " << points.size() << " points, "
	  << "clustering method found " << clusters.size() 
	  << " and took " << elapsedTime.count() << " milliseconds" 
	  << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int index: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[index][0],points[index][1],0));
  		renderPointCloud(viewer, clusterCloud, "cluster"+std::to_string(clusterId), colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer, cloud,"data"); // render if no seperate cluster were found
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	*/
}


/**
 * Recursive method
 * Receives a point index that is determined to be nearby.
 *
 */
void clusterHelper(
        int index,
        const std::vector<std::vector<float>>& points,
        std::vector<int> cluster,
        std::vector<bool> processed,
        KdTree* tree,
        float distanceThreshold)
{
    processed[index] = true;
    cluster.push_back(index); // add this point index as it is already associated with this cluster

    // find points that are close.
    std::vector<int> nearest = tree->search( points[index], distanceThreshold);

    for( int nearbyIndex: nearest)
    {
        if( !processed[nearbyIndex] )
        {
            // the nearby point has not been processed yet for this cluster
            clusterHelper(nearbyIndex, points, cluster, processed, tree, distanceThreshold);
        }
    }
}