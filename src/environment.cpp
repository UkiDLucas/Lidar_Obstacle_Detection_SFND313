/*!
 *  \brief     Enviroment
 *  \details   Enviroment
 *  \author    Aaron Brown https://www.linkedin.com/in/awbrown90/
 *  \author    Uki D. Lucas https://www.linkedin.com/in/ukidlucas/
 *  \date      August 1, 2019
 *  \bug       TBD
 *  \warning   TBD
 *  \copyright code_reuse_license.md
 *  see: http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
 */

#include "sensors/lidar.h"
#include "processPointClouds.h"
#include "pcl/point_types.h"

Color colorBlack      = Color(0.0, 0.0, 0.0); // black background
Color colorRed      = Color(1.0, 0.0, 0.0); // boxes
Color colorGray     = Color(0.5, 0.5, 0.5);
Color colorGreen    = Color(0.0, 1.0, 0.0); // road plane
Color colorBlue     = Color(0.0, 0.0, 1.0);
Color colorViolet   = Color(1.0, 0.0, 1.0);
Color colorTeal     = Color(0.0, 1.0, 1.0);
Color colorPink     = Color(0.8, 0.3, 0.3);
Color colorWhite    = Color(1.0, 1.0, 1.0);
Color colorOlive    = Color(0.5, 0.5, 0.2); // obstacles
// TODO figure out more colors, I have about 12 different clusters

std::vector<Color> colors = { colorBlue, colorTeal, colorViolet, colorGray, colorPink, colorOlive };

// IMPLEMENTATION

PointCloud<PointXYZI>::Ptr &
preprocessDownsizePointCloud(ProcessPointClouds &pointProcessor, PointCloud<PointXYZI>::Ptr &inputCloud);

/**
 * This method takes a single frame of Point Cloud Data,
 * and processes it,
 * then renders it.
 * @param viewer PCL PCLVisualizer
 * @param pointProcessor PCL ProcessPointClouds<pcl::PointXYZI>
 * @param inputCloud the raw PCD before processing
 */
void processSingleFrame(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds pointProcessor, // do not re-create every time
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud) // inputCloud vary from frame to frame
{
    // START TIMER
    auto startTime = std::chrono::steady_clock::now();
    inputCloud = preprocessDownsizePointCloud(pointProcessor, inputCloud);

    //renderPointCloud(viewer, inputCloud, "inputCloud");


    // FIND THE ROAD PLANE
    std::unordered_set<int> roadPlanePointIndices = pointProcessor.findPlaneUsingRansac3D(inputCloud,100,0.2);
    //std::cout << "FOUND roadPlanePointIndices " << roadPlanePointIndices.size () << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr onRoadPlanePoints(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstaclesPointCloud(new pcl::PointCloud<pcl::PointXYZI>());

    // SEPARATE THE ROAD FROM THE OBSTACLE POINT CLOUD
    for(int index = 0; index < inputCloud->points.size(); index++)
    {
        pcl::PointXYZI point = inputCloud->points[index];
        if(roadPlanePointIndices.count(index))
            onRoadPlanePoints->points.push_back(point);
        else
            obstaclesPointCloud->points.push_back(point);
    }

    // RENDER CLOUDS
    renderPointCloud(viewer, onRoadPlanePoints, "road plane", colorGreen);
//    renderPointCloud(viewer, obstaclesPointCloud, "obstaclesPointCloud", colorWhite);



    float clusterTolerance = 0.4;   // e.g. less than 1.5 divides the car in two
    int minClusterSize = 10;        // weed out the single point outliers (i.e. gravel)
    int maxClusterSize = 650;       // my biggest car is 278 points

    // SEPARATE THE OBSTACLE CLOUD INTO INDIVIDUAL OBSTACLE POINT CLOUDS
//    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> uniqueClustersClouds =
//            pointProcessor.pclClustering(obstaclesPointCloud, clusterTolerance, minClusterSize, maxClusterSize);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> uniqueClustersClouds
        = pointProcessor.separatePointCloudClusters(
                    obstaclesPointCloud);

//    cout << "separatePointCloudClusters() returned " << uniqueClustersClouds.size() << " uniqueClustersClouds" << endl;
//    pointProcessor.numPoints(cluster);
    int clusterId = 0;
    int colorId = 0; // I have less colors than object, so I need to recycle

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : uniqueClustersClouds)
    {
//        std::cout << "Iterating thru cluster " << clusterId
//            << " with " << cluster->points.size() << " points"
//            << endl;
        // RENDER ONE OBSTACLE
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[colorId]);
        // RENDER A BOX AROUND ONE OBSTACLE
        if(cluster->points.size() > 5){ //TODO Bound only LARGE OBJECTS, in production I might re-think this
            Box box = pointProcessor.boundingBox(cluster);
            renderBox(viewer, box, clusterId, colorRed, 0.5);
        }

        ++clusterId;
        if (colorId < colors.size() - 1)
            ++colorId;
        else
            colorId = 0;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Processing a single frame took " << elapsedTime.count() << " milliseconds, or " << (1000/elapsedTime.count()) << " FPS" << std::endl;

}

PointCloud<PointXYZI>::Ptr &
preprocessDownsizePointCloud(ProcessPointClouds &pointProcessor, PointCloud<PointXYZI>::Ptr &inputCloud) {/**
 * DECREASE THE AMOUNT OF POINTS WE ARE PROCESSING
 * In practice, the area processed and the zoom could be adjusted from frame-to-frame.
 * The interesting sub-parts of the frame could be insulated into separate point clouds
 * and processed at higher resolution, i.e. tracking pedestrians, posts, street work obstacles
 * The rest of the cloud could be tracked in low resolution i.e. cars driving behind us.
 */
    float downSampleTo = 0.20; // meters e.g. 6cm = 0.06
    pointProcessor.downsizeUsingVoxelGrid(inputCloud, downSampleTo);

    // REMOVE THE VEHICLE ROOF POINTS
    pointProcessor.cropVehicleRoof(inputCloud);

    // CROP REGION (THE HORIZON)
// TODO crop the obstacles AFTER segmentation
    float seeForward    = 40.0; // in reality as much as 250m
    float seeBackwards  = 10.0; // meters
    float seeRight      = 5.0; // meters, right-hand side driving e.g. 8.0
    float seeLeft       = 7.2; // meters, right-hand side driving e.g. 13.0
    float seeUp         = 2.0; // meters, from the roof of the car e.g. 3.0
    float seeDown       = 2.0; // meters, from the roof of the car
    Eigen::Vector4f minRange = Eigen::Vector4f (-seeBackwards, -seeRight, -seeDown, 1);
    Eigen::Vector4f maxRange = Eigen::Vector4f (seeForward, seeLeft, seeUp, 1);
    pointProcessor.cropRegion(inputCloud, minRange, maxRange);
    return inputCloud;
}


/**
 * The viewer passed in as a reference, any changes will persist outside of this funciton
 * setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
 * @param setAngle
 * @param viewer
 */
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (colorBlack.r, colorBlack.g, colorBlack.b); // seting background color
    
    // set camera position and angle
    viewer->initCameraParameters();

    int distance = 16; // distance away in meters
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


/**
 *
 * @param argc
 * @param argv
 * @return
 */
//template<typename PointT> //error: 'main' cannot be a template
int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    // ProcessPointClouds<PointT> pointProcessor; //error: 'main' cannot be a template
    ProcessPointClouds pointProcessor;

    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("src/sensors/data/pcd/data_1/0000000000.pcd");
    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("src/sensors/data/pcd/data_1/");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    while (!viewer->wasStopped() )
    {
        // clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load PCD
        inputCloud = pointProcessor.loadPcd((*streamIterator).string()); // dereference the iterator to get path
        //  obstacle detection process
        processSingleFrame(viewer, pointProcessor, inputCloud);

        // move to the next frame
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    } 
}




