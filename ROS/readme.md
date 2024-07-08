# ILCC_ROS  

ILCC’s ROS package

## Subscribed topics  

/point_cloud_topic

/image_topic

## Published topics

### Registration Results

/visualization_marker_array  
/points_corners  
/points_segs  
/points_raw  

## Usage

### Installation

`mkdir catkin_ws/src/ilcc-ros`  
`cd catkin_ws/src/ilcc-ros`  
`git clone https://github.com/hai12hai12/lidar.git`  
`cd catkin_ws`  
`catkin_make -DCATKIN_WHITELIST_PACKAGES="ilcc-ros"`  

### Explanation of files

bin: The file in bin are executable files of python and can be run directly.  
load: ROS Signal Reception and File Storage.  
Registion:File Registration for Alignment.  
Visualization:Visualization of Alignment Results.   
config.yaml: parameter settings    
data.bag: Demo data containing 20 sets of images and their corresponding point clouds, where the image message is compressed.  

### Process

1.Roslaunch load.launch is ready to receive signals for the release of the package, press the spacebar to store the subscribed messages as a file.    
![image](https://github.com/hai12hai12/ILCC-ros/blob/master/readme-files/0001.jpg)  
2.The project subscribes to ponitcloud2 and image messages. Publish pointcloud2 and image messages, if using the demo we provide, the image message needs to be decompressed.    
3.Roslaunch registration.launch for point cloud registration via ILCC.  
![image](https://github.com/hai12hai12/ILCC-ros/blob/master/readme-files/0001.rviz.png)  
4.Roslaunch rviz.launch to visualize the registration results.  
![image](https://github.com/hai12hai12/ILCC-ros/blob/master/readme-files/0001_cal_backproj.jpg)  
