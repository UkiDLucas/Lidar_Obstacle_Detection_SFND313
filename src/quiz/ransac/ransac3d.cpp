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
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud, 
	int maxIterations, 
	float distanceTreshhold)
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
		// Declare an unordered set of random 3 points.
		std::unordered_set<int> inliers;

		// Randomly select 3 inliers points from the inputPointCloud
		while(inliers.size() < 3)
		{
			// mod of cloud's size: results between 0 and the cloud size:
			// we are using a set so we will not be able to pick the same point.
			inliers.insert(rand()%(inputPointCloud->points.size()));
		}
		// Declare points (x1, y1, z3), (x2, y2, z2), (x3, y3, z3)
		// they  have XYZ coordinates and there are 3 sets needed for a plane
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		// start selecting from the begining of the inliers set of 3
		auto itr = inliers.begin(); 

		// grab the XYZ values from the cloud that correspond to the randomly selected inliers
		x1 = inputPointCloud->points[*itr].x;
		y1 = inputPointCloud->points[*itr].y;
		z1 = inputPointCloud->points[*itr].z;
		itr++;
		x2 = inputPointCloud->points[*itr].x;
		y2 = inputPointCloud->points[*itr].y;
		z2 = inputPointCloud->points[*itr].z;
		itr++;
		x3 = inputPointCloud->points[*itr].x;
		y3 = inputPointCloud->points[*itr].y;
		z3 = inputPointCloud->points[*itr].z;

		// PLANE: Ax+By+Cz+D=0
		// v1 = < x2-x1, y2-y1, z2-z1 >
		// v2 = < x3-x1, y3-y1, z3-z1 >

		// Find normal vector to the plane by taking cross product of v1 x v2:
		// Product of vectors v1, v2
		// v1×v2 = <i, j, k>
		float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

		// then,
		// i(x-x1)+j(y-y1)+k(z-z1) = 0 
		// ix + jy + kz -( ix1 + jy1 + kz1 ) = 0 

		float A = i;
		float B = j;
		float C = k;
		// Calculate the coefficient D
		float D = -(i*x1+j*y1+k*z1);
		

		// Iterate thru all the cloud points:
		for(int index = 0; index < inputPointCloud->points.size(); index++)
		{
			// if the inliers alrady contain the point index
			if(inliers.count(index) > 0)
			{
				// then continue to the next iteration of the for-loop
				continue;
			}
			// Get the point for a given index, to extract Z coordinates:
			pcl::PointXYZ point = inputPointCloud->points[index];
			float xTest = point.x;
			float yTest = point.y;
			float zTest = point.z;

			// Calculate distance of the point:
			// fabs = float absolute value
			// LINE:
			float d = fabs(A * xTest + B * yTest + C * zTest + D) / sqrt(A*A + B*B + C*C);

			// if distance is less, or equal to the threshhold:
			if( d <= distanceTreshhold)
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
