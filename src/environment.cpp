/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    /**
     * This flag (true) allows you to render the whole scene (highway, cars),
     * or just the cloud (false) - point cloud and rays
     */
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    /**
     * TODO:: Create lidar sensor 
     * src/sensors/lidar.h header file
     * Lidar pointer object on the heap using "new" keyword, 
     * stack has only about 2MB, 
     * however it takes longer to look up objects on the heap.
     * double setGroundSlope = 0.0 for flat surface
     */
    Lidar* lidar = new Lidar(cars, 0.0);

    /** 
     * scan() does the ray casting
     * scan() function does not take any parameters 
     * scan() generates the Pointer object PointCloud of type PointXYZ named inputCloud
     * The Pointer - 32 bit integer that contains the memory address of your point cloud object
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan(); 

    /** 
     * void renderRays is in src/render/render.cpp 
     * lidar->position is the ORGIN where the rays are cast from.
     * see: Vect3 position in lidar.h line 78
     */
    //renderRays(viewer, lidar->position, inputCloud);

    /**
     * This method renders just the point cloud,
     * this is what you would receive from the Lidar.
     */
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    /** 
     * TODO:: Create point processor
     */
    //auto pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor; // on the stack
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = 
        new ProcessPointClouds<pcl::PointXYZ>(); // on heap

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> 
    int iterations = 1000;
    float distanceTreshhold = 0.05; // meters
    segmentCloud = pointProcessor->SegmentPlane(inputCloud, iterations, distanceTreshhold);
    
    renderPointCloud(viewer, segmentCloud.first, "obstructions", Color(1,0,0)); // RED
    renderPointCloud(viewer, segmentCloud.second, "road plane", Color(0,1,0)); // GREEN
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