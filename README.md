# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.



# Uki's Class Progress

#### Thursday, July 18, 2019

Total class time spent: ~26 hours.

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





##### Applying Euclidean Clustering and KD Tree



Coming soon.






























## Installation

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



