# Robot Package Template

This is a GitHub template made for the ITS Main Vehicle. You can make your own copy by clicking the green "Use this template" button or make a fork to your own repository. Or you can clone a copy of this repo into your local system.

## 0. Pre-requisite
### 0.1. System Requirements
- Ubuntu 22.04.
- ROS2 Humble.
- Laptop with 8GB Ram, 50GB Storage.

### 0.2. Install ROS2 and Related Softwares
1. Follow the guide below to install ROS2 Humble.

**ROS 2 Install Guide**: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2. Follow the guide below to install Gazebo Ignition Fortress.

**Gazebo Ignition Fortress install**: https://gazebosim.org/docs/fortress/install_ubuntu/

3. Follow the guide below to install Ros-Gazebo Pairing

**ROS2 Gazebo Bridge**: https://gazebosim.org/docs/fortress/ros_installation/

4. Follow the guide below to install Nav2:

**NAV2**: https://docs.nav2.org/getting_started/index.html  
Note: We're using ROS2 humble distribution so when installing make sure use the command under "***Iron and older***"

### 0.3. Other Packages You May Need

```
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-gz-ros2-control ros-humble-gz-ros2-control-demos
```


## 1. Simulation
### 1.1. Source ROS2 setup file
You need to run this command everytime you open up a new terminal 
```
source /opt/ros/humble/setup.bash
```
Note: The common trick is copy this command to your ~/.bashrc file. 

### 1.2. Build and Source the package  

```
colcon build
source install/setup.bash
```

### 1.3. Launch Simulation 
```
ros2 launch its_main_vehicle launch_sim.launch.py 
```
### 1.4. Manual Control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped
```