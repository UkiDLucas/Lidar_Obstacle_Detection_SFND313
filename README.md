# Sensor Fusion Self-Driving Car Course



### Welcome to the Sensor Fusion course for [ADAS](https://en.wikipedia.org/wiki/Advanced_driver-assistance_systems) and autonomous driving.

<img src="media/tracking objects.gif" />





In this course we will be talking about sensor fusion, which is the process of **taking data from multiple sensors and combining it to give us a better understanding of the world around us**. We will mostly be focusing on three sensors, lidar, camera and radar. By the end we will be fusing the data from these sensors to track multiple cars on the road, estimating their positions and speed.

Point Cloud Library (PCL) allows us to **divide an unorganized collection of 3D points in order to process the subsections rapidly**. When Lidar point clouds are **combined with camera images, this technique allows us to positively identify objects and provide their distance**, while Radar allows us to track their speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently still very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resolution imaging with radar's ability to measure velocity of objects we can get a better understanding of the surrounding environment than we could using one of the sensors alone.



# Class Progress

## Friday, Aug 9, 2019

Total class time spent: start ~43 hours, end TBD hours.

- 



## Wednesday, July 31, 2019

Total class time spent: start ~40 hours, end ~43 hours.

- Implemented my own 3D KD Tree 





## Wednesday, July 31, 2019

Total class time spent: start ~37 hours, end ~40 hours.

- Implemented Road Plane separation from the Obstacle Cloud
- Bounding Boxes
- The project runs well 



## Tuesday, July 30, 2019

Total class time spent: start ~36 hours, end ~37 hours.

### "Lidar Final Project"

https://www.youtube.com/watch?v=lGbHW8SMu24

In this project you will take everything that you have learned for processing point clouds, and use it to detect car and trucks on a narrow street using lidar. The detection pipeline should follow the covered methods, filtering, segmentation, clustering, and bounding boxes. Also the **segmentation, and clustering methods should be created from scratch** using the previous lesson’s guidelines for reference.

- Final project



[![YouTube video](media/ObstacleDetectionFPS.gif)](https://www.youtube.com/watch?v=lGbHW8SMu24)





## Monday, July 29, 2019

Total class time spent: start ~35 hours, end ~36 hours.

### Lesson 4 "Working with Real PCD"

- Downsampling/filtering the Point Cloud Data
-  The streaming point cloud (animation) is working!







## Saturday, July 27, 2019

Total class time spent: start ~34 hours, end ~35 hours.

### Lesson 4 "Working with Real PCD"

- Rendring the cloud

<img src="media/Real PCD 2019-07-27 at 7.22.15 PM.png" />









## Tuesday, July 23, 2019

Total class time spent: start ~33 hours, end ~34 hours.

### Lesson 4 "Working with Real PCD"

- Just started, but it is not rendering









### Lesson 3.9 "Bounding Boxes"

- Implemented code

<img src="media/environment 2019-07-23 at 4.25.55 PM.png" />



### Lesson 3.8 "Euclidean pclClustering"

- Implemented euclideanCluster and clusterHelper, but it finds 11 clusters for 11 points,











## Monday, July 22, 2019

Total class time spent: start ~31.5 hours, end ~33 hours.

- Cleaned up searchHelper, the bug is in the way I pass in resultIds: 
  - std::vector<int>& resultIds - fixed





## Sunday, July 21, 2019

Total class time spent: start ~30 hours, end ~31.5 hours.

Read few articles on Machine Learning on MacOS, CUDA (NVidia) vs OpenCL (AMD) GPU.

```
$ pip install -U plaidml-keras
Requirement already satisfied, skipping upgrade: keras==2.2.4 in /anaconda2/lib/python2.7/site-packages (from plaidml-keras) (2.2.4)
Successfully installed plaidml-0.6.3 plaidml-keras-0.6.3
$ plaidml-setup
```

- Installing [PlaidML](https://vertexai-plaidml.readthedocs-hosted.com/en/latest/index.html)

0815 - 0945

### Lesson 3.7 Searching Points in KD-Tree

- fixing bugs
- Cleaning and adding documentation to the code
- Still getting bad result pclClustering found 0 and took 0 milliseconds

### 











## Saturday, July 20, 2019

Total class time spent: start ~29.5 hours, end ~30 hours.

- I have re-read documentation about Euclidean Cluster Extraction: http://pointclouds.org/documentation/tutorials/cluster_extraction.php
- Added documentation to my project: https://github.com/UkiDLucas/SFND313_Lidar_Obstacle_Detection#ukis-class-progress
- Renamed variables in kdtree.h to make code easier to read and understand.









## Friday, July 19, 2019

Total class time spent: ~29.5 hours.

##### Bounding Boxes



<img src="media/Bounding Boxes 2019-07-19 at 7.40.46 AM.png" />



##### KD Tree

The KD Tree is used to drastically speed up a look up of points in space. 

The KD Tree is a form of binary tree where you sort the e.g. points (X,Y), shown below, you take the median of X from all the points and insert it as a root node, then take median of all Y and insert it as first child, then continue recursively.

<img src="media/KD Tree 2019-07-19 at 8.51.21 AM.png" />





## Thursday, July 18, 2019

Total class time spent: ~27 hours.

Today, I was able to implement the first version of RANSAC 3D that fits a plane (e.g. road surface) to a Point Cloud data. 

<img src="media/RANSAC 3D segmentation 2019-07-18 at 4.03.06 PM.png" width="800" />



Another happy milestone is that I am able to run it in Mac OS without Ubuntu (dual boot, or remote)

You can see that the plane (green dots) upper boundary mixes with obstacles (red dots), the following image is after adjusting iterations to 1,000 and the distance threshold down to 0.05 meters (5 cm).

Happy medium is somewhere in between.



<img src="media/RANSAC 3D 2019-07-18 at 4.50.36 PM.png"  />







<img src="media/RANSAC 3D equations 2019-07-18 at 9.18.01 AM.png" />



```
SFND313_Lidar_Obstacle_Detection/src/quiz/ransac/ $ make clean && make && ./quizRansac
```





##### Applying Euclidean pclClustering and KD Tree

I am correctly detecting all 3 cars and the road underneath.

```
float clusterTolerance = 1.5; *// e.g. less than 1.5 divides the car in two*
int minClusterSize = 1; *// weed out the single point outliers (i.e. gravel)*
int maxClusterSize = 500; *// my biggest car is 278 points*
```



<img src="media/pclClustering 2019-07-18 at 5.35.06 PM.png"  />























# Further Reading

- Euclidean Cluster Extraction: http://pointclouds.org/documentation/tutorials/cluster_extraction.php
- pcl Namespace Reference: http://docs.pointclouds.org/1.0.0/namespacepcl.html
- Distance between Point and a Line: https://brilliant.org/wiki/dot-product-distance-between-point-and-a-line/
- Extracting indices from a PointCloud: http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices









# Installation

### Ubuntu (Parallels on MacBook Pro, Radeon Pro 560 4 GB)

See my blog post:
https://ukidlucas.blogspot.com/2019/07/ubuntu-cmake.html

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew

1. install [homebrew](https://brew.sh/)

2. update homebrew 

   ```bash
   $> brew update
   ```

3. add  homebrew science [tap](https://docs.brew.sh/Taps) 

   ```bash
   $> brew tap brewsci/science
   ```

4. view pcl install options

   ```bash
   $> brew options pcl
   ```

5. install PCL 

   ```bash
   $> brew install pcl
   ```



## Compilation on MacOS

```
$ pwd

SFND313_Lidar_Obstacle_Detection/build
build $ cmake ../CMakeLists.txt
```

Compilation results in ERROR:

```
-- Checking for module 'glew'

--   No package 'glew' found

CMake Error at /usr/local/share/pcl-1.9/PCLConfig.cmake:58 (message):

  simulation is required but glew was not found
```

##### FIX for the "missing glew" problem (No package 'glew' found)

```
build $ cd ..

(turi) uki  13:00 SFND313_Lidar_Obstacle_Detection $ edit CMakeCache.txt 
```

EDIT lines (~279) to look similar to:



```
//GLEW library for OSX
GLEW_GLEW_LIBRARY:STRING=/usr/local/Cellar/glew/2.1.0

//GLEW include dir for OSX
GLEW_INCLUDE_DIR:STRING=/usr/local/Cellar/glew/2.1.0
```



```
SFND313_Lidar_Obstacle_Detection $ cd build/

(turi) uki  13:03 build $ cmake ../CMakeLists.txt

...

-- Configuring done

-- Generating done

-- Build files have been written to: /Volumes/DATA/_Drive/_REPOS/SFND313_Lidar_Obstacle_Detection


build $ cd ..
(turi) uki  13:09 SFND313_Lidar_Obstacle_Detection $ make clean && make
```



#### Prebuilt Binaries via Universal Installer

http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)

