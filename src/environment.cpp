/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <pcl/point_cloud.h>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
// viewer is passed in as a reference, any changes will persist outside of this funciton
{
    // create cars
    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{    /**
     * This flag (true) allows you to render the whole scene (highway, cars),
     * or just the cloud (false) - point cloud and rays
     */
    bool renderScene = true;
    bool render_road_plane = true;
    bool render_obstructions = true;
    bool render_clusters = true;
    bool render_box = true;
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    std::vector<Car> cars = initHighway(renderScene, viewer);
    double setGroundSlope = 0.0;
    Lidar* lidar = new Lidar(cars, setGroundSlope);

    /** 
     * scan() does the ray casting
     * scan() function does not take any parameters 
     * scan() generates the Pointer object PointCloud of type PointXYZ named inputCloud
     * The Pointer - 32 bit integer that contains the memory address of your point cloud object
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarScanCloud = lidar->scan(); 
    //typename pcl::PointCloud<PointT>::Ptr lidarScanCloud = lidar->scan(); // error: no type named 'Ptr' in the global namespace

    /** 
     * void renderRays is in src/render/render.cpp 
     * lidar->position is the ORGIN where the rays are cast from.
     * see: Vect3 position in lidar.h line 78
     */
    //renderRays(viewer, lidar->position, lidarScanCloud);

    /**
     * This method renders just the point cloud,
     * this is what you would receive from the Lidar.
     */
    //renderPointCloud(viewer, lidarScanCloud, "lidarScanCloud");

    /** 
     * TODO:: Create point processor
     */
    //auto pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor; // on the stack
    ProcessPointClouds<pcl::PointXYZ>* 
        pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); // on heap

    // Segmenting out the road PLANE
    int iterations = 1000;
    float distanceTreshhold = 0.1; // meters, increase by 0.05 if road surface (i.e. gravel?) is misimpreted as obstacles.

    std::pair<
        pcl::PointCloud<pcl::PointXYZ>::Ptr, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr> 
        segmentPlaneCloudPair = pointProcessor->SegmentPlane(lidarScanCloud, iterations, distanceTreshhold);

    // render layers in order of importance of what you want to see in the final view        
    if(render_road_plane)
        renderPointCloud(viewer, segmentPlaneCloudPair.second, "road plane", Color(0,1,0)); // GREEN
    
    if(render_obstructions)
        renderPointCloud(viewer, segmentPlaneCloudPair.first, "obstructions", Color(1,0,0)); // RED


    pcl::PointCloud<pcl::PointXYZ>::Ptr allObstructionsCloud = segmentPlaneCloudPair.first;
    
    float clusterTolerance = 1.5; // e.g. less than 1.5 divides the car in two
    int minClusterSize = 1; // weed out the single point outliers (i.e. gravel)
    int maxClusterSize = 500; // my biggest car is 278 points

    // collection of clusters
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> 
        uniqueClustersClouds = pointProcessor->Clustering(allObstructionsCloud, clusterTolerance, minClusterSize, maxClusterSize);

 

    //orange red Color(255,69,0)
    //dark orange Color(255,140,0)
    //orange Color(255,165,0)
    //gold Color(255,215,0)

    int clusterId = 0;
    std::vector<Color> colors = {Color(0,0,1), Color(255,153,51), Color(255,215,0), Color(255,140,0)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : uniqueClustersClouds)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
        }

        if(render_box)
        {
            // member reference type 'ProcessPointClouds<pcl::PointXYZ> *' is a pointer; did you mean to use '->'?
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
}

void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointYXZI>* pointProcessor,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster
){
    
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer) 
// viewer passed in as a reference, any changes will persist outside of this funciton
{

    viewer->setBackgroundColor (0, 0, 0); // seting background color
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer); // passing in viewer 

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}