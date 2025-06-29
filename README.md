## Robot Package Template

This is a GitHub template made for the ITS Main Vehicle. You can make your own copy by clicking the green "Use this template" button or make a fork to your own repository.


## 1. Simulation
### 1.1. Launch Simulation 
```
ros2 launch its_main_vehicle launch_sim.launch.py 
```
### 1.2. Manual Control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped
```