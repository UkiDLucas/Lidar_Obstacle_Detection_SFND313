/* \author Aaron Brown, then Uki D. Lucas */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h" // 2D

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(
	Node* node, 
	pcl::visualization::PCLVisualizer::Ptr& viewer, 
	Box window, 
	int& iteration, 
	uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),
				pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),
				pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

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
	cluster.push_back(index); // add this point index as it is alrady associated with this clusterr

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

/**
 * returns the list of cluster indices
 * To perform the clustering, 
 * iterate through each point in the cloud and keep track of which points have been processed already.
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
euclideanCluster(
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
			// move to the next point
			i++;
			continue; 
		}
		// This point was NOT processed.
		// Create a new cluster.
		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceThreshold);
		clusters.push_back(cluster);
		i++; // move to the next point
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
  	
  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, distanceThreshold);

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
  		renderPointCloud(viewer, cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
