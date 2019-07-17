/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	// corrected the path to pcd
	return pointProcessor.loadPcd("../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> 
Ransac(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	int maxIterations, 
	float distanceTol)
{
	// Declare an unordered set of the best results:
	std::unordered_set<int> inliersResult;

	// Seed the random function with time:
	srand(time(NULL));
	
	/** 
	 * Looping down from maxIterations
	*/
	while(maxIterations --)
	{
		// Declare an unordered set of random 2 points.
		std::unordered_set<int> inliers;

		// Randomly select 2 points from the cloud set
		while(inliers.size() < 2)
		{
			// mod of cloud's size: results between 0 and the cloud size:
			// we are using a set so we will not be able to pick the same point.
			inliers.insert(rand()%(cloud->points.size()));
		}
		// Declare values for points (x1,y1), (x2, y2)
		float x1, y1, x2, y2;
		auto itr = inliers.begin(); // start from the begining of the set (of 2)

		// grab the values from the cloud:
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		// Use linear equasions to calculate the coefficients:
		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2-x2*y1);

		// Iterate thru all the cloud points:
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// if the inliers alrady contain the point index, then continue:
			if(inliers.count(index) > 0)
			{
				continue;
			}
			// Get the point for a given index, to extract Z coordinates:
			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;

			// Calculate distance of the point:
			// fabs = float absolute value
			float d = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b);

			// if distance is less, or equal to the threshhold:
			if( d<= distanceTol)
				inliers.insert(index);
		}

		// Pick bigger set of points (line fitting more points)
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}

	}

	//TODO: add timer
	
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D(); // Used with RANSAC PLANE surface fitting
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
