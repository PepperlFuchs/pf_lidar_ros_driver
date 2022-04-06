## ROS drivers for R2000 and R2300 laser scanners

![Build Status](https://github.com/PepperlFuchs/pf_lidar_ros_driver/actions/workflows/main.yml/badge.svg)

**Required platform:**  
Ubuntu 20.04 and ROS Noetic
  
**Clone the repository:**  
Clone the repository in the `src` folder of your ROS workspace
```
git clone --branch=main https://github.com/PepperlFuchs/pf_lidar_ros_driver.git
```
  
**Install the missing dependencies:**  
```
cd <path/to/workspace>
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```
  
**Build the workspace:**  
```
cd <path/to/workspace>
source /opt/ros/noetic/setup.bash
catkin build
source <path/to/workspace>/install/setup.bash
```
  
**Usage:**  
Now you are ready to use the driver. Make the necessary power and ethernet connections. Make sure your computer's IP address is on the same subnet mask as that of the device. Change the `scanner_ip` argument in the respective launch file as necessary. You can now launch one of the drivers in the following manner:  
```
roslaunch pf_driver r2000.launch
```
OR
```
roslaunch pf_driver r2300.launch
```
With R2300, the term scan refers to a contiguous group of measurements spanning one particular horizontal circular
sector. Depending on the orientation of the mirrors on the cube, the scans may be taken in the same or slightly different
layers.  
  
In current sensors, all four mirrors are inclined slightly differently so that scans are taken at the following vertical
angle (relative to the mounting plane). Note that the layers are numbered in the order they are scanned during one
turn. This is (yet) not strictly from bottom to top:

| **Layer index** | **Angle** | **Description** |
|-----------------|-----------|-----------------|
|0 |-4.5°|bottom (connector side)|
|1 |-1.5° | - |
|2 |+4.5° | top |
|3 |+1.5° | - |
