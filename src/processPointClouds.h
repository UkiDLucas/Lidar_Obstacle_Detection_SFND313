/*!
 *  \brief     PCL lib Functions for processing point clouds
 *  \details   PCL lib Functions for processing point clouds
 *  \author    Aaron Brown https://www.linkedin.com/in/awbrown90/
 *  \author    Uki D. Lucas https://www.linkedin.com/in/ukidlucas/
 *  \date      August 1, 2019
 *  \bug       TBD
 *  \warning   TBD
 *  \copyright code_reuse_license.md
 */

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include "kdtree3D.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

class ProcessPointClouds {
public:

    // constructor
    ProcessPointClouds();
    // destructor
    ~ProcessPointClouds();

    //void numPoints(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    void cropVehicleRoof( pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloud );



    void downsizeUsingVoxelGrid(typename pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloud, float leafSize);

    void cropRegion(
            typename pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud,
            Eigen::Vector4f minRange,
            Eigen::Vector4f maxRange);


    std::unordered_set<int> findPlaneUsingRansac3D(
                typename pcl::PointCloud<pcl::PointXYZI>::Ptr inputPointCloud,
                int maxIterations,
                float distanceTreshold);

    //std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> separate2Clouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    //std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> pclSegmentPlane(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold);

    //std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> pclClustering(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> separatePointCloudClusters(const pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud);

    pcl::PointXYZI extractPointFromPointCloud(
            const vector < float > point,
            const std::__1::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> cloudPoints
            ) const;

    pcl::PointXYZI extractPointFromPointCloudAtIndex(
            const int index,
            const std::__1::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> cloudPoints
            ) const;



    Box boundingBox( pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);

    //void savePcd(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string file);

    pcl::PointCloud<pcl::PointXYZI>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);


private:
    KdTree3D *
    populateTree(
            std::vector<std::vector<float>> &pointsXYZ
            ) const;

private:
    std::vector<std::vector<float>>
    convertCloudToPoints(
            const std::__1::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> cloudPoints
            ) const;

private:
    void
    recursivelyPopulateClusterWithNearbyPoints(
            vector< vector< float > > cluster,
            vector< float > point,
            vector< bool > availablePoints,
            KdTree3D *tree,
            const float distanceThreshold);
};
#endif /* PROCESSPOINTCLOUDS_H_ */