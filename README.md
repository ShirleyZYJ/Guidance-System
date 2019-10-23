# Guidance-System
Building automated guidance system using robot-mounted or handheld zed-mini.
![Girl](https://github.com/ghost-60/Guidance-System/blob/master/sample/sample.png)

### Current Status
- Rtabmap ROS implimentaion is used and integrated with move_base package for path planning.
- Points in the point cloud generated by Rtabmap is labelled with semantic information.
- Using the segmentation images and depth map, a text->voice guided program is made.

### Installation of software
- First setup the Jetson Xavier using the "SDKManager" and install all jetson SDK components.
- Install Zed SDK for Jetson(Follow the website)
https://www.stereolabs.com/developers/nvidia-jetson/

- Install ROS melodic
http://wiki.ros.org/melodic/Installation/Ubuntu

- Install ZED-ROS-Wrapper
https://github.com/stereolabs/zed-ros-wrapper

- Install cv_bridge in the ros workspace
Set the following variables while calling cmake
```
$ cmake 
-DPYTHON_EXECUTABLE=/usr/bin/python3
-DPYTHON_INCLUDE_DIR=/usr/include/python3.6m 
-DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so ../
```

- Setup the "semantic-segmentation-pytorch"
Install python packages - pytorch, scipy(v1.1.0), numpy, torchvision, yacs, tqdm
 Note: Newer versions of scipy have some issue.

- Build the Rtabmap package (Dont put this in workspace)
https://github.com/introlab/rtabmap/wiki/Installation

- Build Rtabmap_ros 
Put the rtabmap_ros && visguide package in your ros workspace and run "catkin_make"

### Usage

1. Run semantic segmentaion
```
$ rosrun visguide segment.py
```
2. To run the rtabmap and zed-mini camera 
```
$ roslaunch visguide visguide.launch
```

Note: To run the rtabmap and zed camera seperately run (change the package namespace)
```
$ roslaunch visguide rtabmap.launch
$ roslaunch visguide zedm.launch 
```

3. To run the path-planning and text to voice command
```
$ rosrun visguide visguide_node2
```

4. For visualisation
```
$ rviz
```
