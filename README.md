## ROS drivers for R2000 and R2300 laser scanners

![Build Status](https://github.com/PepperlFuchs/pf_lidar_ros_driver/actions/workflows/main.yml/badge.svg)

**Required platform:**  
Ubuntu 18.04 / ROS Melodic OR Ubuntu 20.04 / ROS Noetic  

To use `catkin_tools` instead of `catkin_make` for building from source:
```
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

For more details, refer https://catkin-tools.readthedocs.io/en/latest/installing.html
  
**Clone the repository:**  
Clone the repository in the `src` folder of your ROS workspace
```
git clone --branch=main https://github.com/PepperlFuchs/pf_lidar_ros_driver.git
```
  
**Install the missing dependencies:**  
```
export ROS_DISTRO=melodic OR export ROS_DISTRO=noetic
cd <path/to/workspace>
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=ROS_DISTRO -y
```
  
**Build the workspace:**  
```
cd <path/to/workspace>
source /opt/ros/ROS_DISTRO/setup.bash
```
If `catkin_tools` is installed as shown above:
```
catkin build
source <path/to/workspace>/install/setup.bash
```
Instead, to use `catkin_make`:
```
catkin_make
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
