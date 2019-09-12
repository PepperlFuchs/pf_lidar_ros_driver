ROS drivers for R2000 and R2300 laser scanners

**Required platform:**  
Ubuntu 18.04 and ROS Melodic
  
**Clone the repository:**  
Clone the repository in the `src` folder of your ROS workspace
```
git clone --branch=master https://github.com/ipa320/pepperl-fuchs.git
```
  
**Install the missing dependencies:**  
```
cd <path/to/workspace>
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
```
  
**Build the workspace:**  
```
cd <path/to/workspace>
source /opt/ros/melodic/setup.bash
catkin build
source cd <path/to/workspace>/devel/setup.bash
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

