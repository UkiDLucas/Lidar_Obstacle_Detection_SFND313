/*
 * \author Aaron Brown,
 * \author Uki D. Lucas UkiDLucas@gmail.com @UkiDLucas
 *
 * PCL lib Functions for processing point clouds
 * http://docs.pointclouds.org/1.8.1/classpcl_1_1_point_cloud.html#a86473dec40d705190c6b2c2f795b9f15
 */

#include "processPointClouds.h"
#include "render/render.h"
#include "kdtree3D.h"

using namespace std;
using namespace pcl;

/**  constructor */
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


/**  de-constructor */
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}



/**
 * This method takes to Point Cloud Data,
 * and downsizes it to one point per BLOCK of leafSize.
 *
 * @tparam PointT
 * @param inputCloud
 * @param leafSize - meters, probably no less than 6cm, or 0.06
 * @return downsizedCloud
 */
template<typename PointT>
void ProcessPointClouds<PointT>:: downsizeUsingVoxelGrid(
        typename  pcl::PointCloud<PointT>::Ptr &pointCloud,
        float leafSize)
{
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr downsizedCloud(new pcl::PointCloud<PointT>);
    //std::cout << "name of this VoxelGrid: " << typeid(vg).name() << std::endl;
    vg.setInputCloud(pointCloud);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*pointCloud); // save
}






/**
 *
 * @tparam PointT
 * @param cloud
 * @param minPoint
 * @param maxPoint
 * @return
 */
template<typename PointT>
void ProcessPointClouds<PointT>:: cropRegion(
        typename pcl::PointCloud<PointT>::Ptr &pointCloud,
        Eigen::Vector4f minRange,
        Eigen::Vector4f maxRange)
{
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true); // true - points insider the crop box
    region.setMin(minRange);
    region.setMax(maxRange);
    region.setInputCloud(pointCloud);
    region.filter(*pointCloud); // write results
}



/**
 *
 * @tparam PointT
 * @param pointCloud - passed by reference, any changes will be persisted.
 * @param minRange
 * @param maxRange
 */
template<typename PointT>
void ProcessPointClouds<PointT>::cropVehicleRoof(
        typename pcl::PointCloud<PointT>::Ptr &pointCloud )
{
    typename pcl::PointCloud<PointT>::Ptr returnCloud(new pcl::PointCloud<PointT>);

    // REMOVE VEHICLE ROOF points, they are static and do not add value
    pcl::CropBox<PointT> roof(true);
    // TODO the roof is very specific to this car, consider extracting settings
    roof.setMin(Eigen::Vector4f (-2.5, -1.6, -1.5, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.6, 0.5, 1));
    roof.setInputCloud(pointCloud);
    std::vector<int> indices;
    roof.filter(indices);
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point: indices)
        inliers->indices.push_back(point);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(pointCloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // remove the roof points
    extract.filter(*pointCloud);
}


template<typename PointT>
std::pair<
    typename pcl::PointCloud<PointT>::Ptr, 
    typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::
    separate2Clouds(
    pcl::PointIndices::Ptr inliers, 
    typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obstaclesCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr roadPlaneCloud (new pcl::PointCloud<PointT> ());

    for (int index : inliers->indices) {
        roadPlaneCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstaclesCloud);

    std::cout 
    << "PointCloud representing the planar component: " 
    << roadPlaneCloud->width * roadPlaneCloud->height 
    << " data points." 
    << std::endl;

    std::pair<
            typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
            resultingPairOfPointClouds(obstaclesCloud, roadPlaneCloud);
    return resultingPairOfPointClouds;
}










/**
 * SegmentPlane function
 */
template<typename PointT>
std::pair<
    typename pcl::PointCloud<PointT>::Ptr, 
    typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::
pclSegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    int maxIterations, 
    float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Declare the segmentation object
    pcl::SACSegmentation<PointT> seg; // any type of Point, e.g., PointXYZ
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    // Set settings for RANSAC plane fitting
    seg.setOptimizeCoefficients(true); // optional
    // mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations); // e.g. 1000
    seg.setDistanceThreshold(distanceThreshold); // e.g. 0.01

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud); // passed to function typename pcl::PointCloud<PointT>::Ptr cloud
    seg.segment(*inliers, *coefficients); // by reference

    // if no inliers points found
    if (inliers->indices.size() == 0) {
        std::cerr << "Could not fit the plane surface for this data set!" << std::endl;
    }

    std::pair<
            typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
            segResult = separate2Clouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::
findUniquePointCloudClusters(const typename pcl::PointCloud<PointT>::Ptr inputCloud)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> uniqueClustersClouds;

    KdTree3D* tree = new KdTree3D;

    std::vector<float> point = {1.0, 2.0};
    tree->insert(point, 1); // works
    std::vector<PointT, Eigen::aligned_allocator<PointT> > points = inputCloud->points;
    std::cout << "findUniquePointCloudClusters inputCloud has  " << points.size() << " points" << std::endl;



    // insert points into the tree
    for (int index = 0; index < points.size(); index++) // iterate thru ever point
    {
        // example point (3.81457,2.23129,-0.890143 - 0.571429)
        // cout << "findUniquePointCloudClusters points for index = " << points[index] << " points" << endl;
        std::tuple<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> pointTuple (points[index]);
        pcl::PointXYZI pointXYZI = std::get<0>(pointTuple); // because C++ is not Python :(
        cout << "findUniquePointCloudClusters point for X = " << pointXYZI.x << " Y =" << pointXYZI.y << endl;

        vector<float> point = {pointXYZI.x, pointXYZI.y, pointXYZI.z};

        // void insert(std::vector<float> point, int pointCloudIndex)
        tree->insert(point, index); // actual point and original index
    }

    uniqueClustersClouds.push_back(inputCloud); // temporarily add whole cloud
    return uniqueClustersClouds;
}






/**
 * http://pointclouds.org/documentation/tutorials/cluster_extraction.php
 * @tparam PointT
 * @param inputCloud
 * @param clusterTolerance
 * @param minClusterSize
 * @param maxClusterSize
 * @return
 */
template<typename PointT>
    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::
pclClustering(
    typename pcl::PointCloud<PointT>::Ptr inputCloud, 
    float clusterTolerance, 
    int minClusterSize, 
    int maxClusterSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec; //TODO check if we can use this
    ec.setInputCloud (inputCloud);

    ec.setClusterTolerance (clusterTolerance); // e.g. 0.02 or 2cm
    ec.setMinClusterSize (minClusterSize); // e.g. 100, if a cluster is really small, it’s probably just noise
    ec.setMaxClusterSize (maxClusterSize); // e.g. 25000, break up very large clusters

    /** 
     * Lesson 3.4
     * Creating the KdTree object for the search method of the extraction
     * KD Tree is a binary tree
     */
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (inputCloud);
    ec.setSearchMethod (tree);
    

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> uniqueClustersClouds;

    int j = 0;
    for (
        std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
        it != cluster_indices.end (); 
        ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back(inputCloud->points[*pit]); //*
            //cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
            //cloud_cluster->points.push_back (uniqueClustersClouds->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "Unique PointCloud contains " << cloud_cluster->points.size () << " data points." << std::endl;
        //std::stringstream ss;
        //ss << "cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
        uniqueClustersClouds.push_back(cloud_cluster);
    }
   

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " 
        << elapsedTime.count() 
        << " milliseconds and found " 
        << uniqueClustersClouds.size() 
        << " uniqueClustersClouds" 
        << std::endl;

    return uniqueClustersClouds;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the uniqueClustersClouds
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}




/**
 * The method takes 3 random points from the Point Cloud Data (PCD),
 * and fits the PLANE to these 3 points,
 * then checks the distance of other points from this PLANE, counting how many remaining points are close enough.
 * This is repeated for maxIterations.
 * The PLANE with most close points wins and is returned.
 * In case of automotive, the largest plan is usually the ROAD.
 *
 * @param inputPointCloud - Point Cloud Data (PCD)
 * @param maxIterations - how many random planes to try
 * @param distanceTreshold
 * @return unordered_set of point indices
 */
template<typename PointT>
std::unordered_set<int>
        ProcessPointClouds<PointT>::findPlaneUsingRansac3D(
        typename pcl::PointCloud<PointT>::Ptr inputPointCloud,
        int maxIterations,
        float distanceTreshold)
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
        std::unordered_set<int> pointsOnThePlane; // a.k.a. inliers

        // Randomly select 3 pointsOnThePlane points from the inputPointCloud
        while(pointsOnThePlane.size() < 3)
        {
            // mod of cloud's size: results between 0 and the cloud size:
            // we are using a set so we will not be able to pick the same point.
            pointsOnThePlane.insert(rand()%(inputPointCloud->points.size()));
        }
        // Declare points (x1, y1, z3), (x2, y2, z2), (x3, y3, z3)
        // they  have XYZ coordinates and there are 3 sets needed for a plane
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        // start selecting from the begining of the pointsOnThePlane (still just 3 of them)
        auto itr = pointsOnThePlane.begin();

        // grab the XYZ values from the input cloud that correspond to the randomly selected pointsOnThePlane
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
        for(int index = 0; index < inputPointCloud->points.size(); index++) {
            // if the random3points already CONTAIN the next point index
            if (pointsOnThePlane.count(index) > 0) {
                continue; // then continue to the next iteration of the for-loop
            }
            // Get the point for a given index, to extract Z coordinates:
            pcl::PointXYZI point = inputPointCloud->points[index];
            float xTest = point.x;
            float yTest = point.y;
            float zTest = point.z;

            // Calculate distance of the point:
            // fabs = float absolute value
            // LINE:
            float distanceToPlane = fabs(A * xTest + B * yTest + C * zTest + D) / sqrt(A * A + B * B + C * C);

            if (distanceToPlane <= distanceTreshold)
            {
                // since this point is on the plane (or close enough), we add it to inliers
                pointsOnThePlane.insert(index);
            }
        }

        // Pick bigger set of points (line fitting more points)
        if(pointsOnThePlane.size() > inliersResult.size())
        {
            inliersResult = pointsOnThePlane;
        }
    }
    //TODO: add timer

    // Return the PLANE that correspond to the biggest set of points on that plane.
    return inliersResult;
}








