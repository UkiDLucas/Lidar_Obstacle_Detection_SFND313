/*
 * \author Aaron Brown,
 * \author Uki D. Lucas UkiDLucas@gmail.com @UkiDLucas
 *
 * PCL lib Functions for processing point clouds
 * http://docs.pointclouds.org/1.8.1/classpcl_1_1_point_cloud.html#a86473dec40d705190c6b2c2f795b9f15
 */

#include "processPointClouds.h"

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
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1)); //TODO pass from outside minRange
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
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
    SeparateClouds(
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

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesCloud, roadPlaneCloud);
    return segResult;
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

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = 
    SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}








template<typename PointT>
    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::
Clustering(
    typename pcl::PointCloud<PointT>::Ptr inputCloud, 
    float clusterTolerance, 
    int minClusterSize, 
    int maxClusterSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    /**
     * http://pointclouds.org/documentation/tutorials/cluster_extraction.php
     */
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
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








