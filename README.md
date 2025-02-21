# ROS2 Robot with Custom Differential Drive plugin and waypoint follower

### Workspace Setup

```bash
git clone https://github.com/Ellakiya15/ROSbot.git
```

### Build the packages

```bash
colcon build
source install/setup.bash
```
### Start Simulation

```bash
ros2 launch custom_bot gazebo.launch.py
```

## Custom_bot package

This package includes a C++ node that listens to Twist messages from /cmd_vel, converts them into RPM values, and publishes them on the topics:

* **/left_wheel_rpm**

* **/right_wheel_rpm**

where wheelbase, wheel_radius and Maximum rpm are dynamically changeable.
And also has **custom plugin** for Differential Drive called **"DiffDrive"**.

### To run Differential Drive Controller 

```bash
ros2 run custom_bot cmd_vel_to_rpm
```

###  To check rpm values

```bash
ros2 topic echo /left_wheel_rpm
ros2 topic echo /right_Wheel_rpm
```
For gazebo
```bash
gz topic -i /left_wheel_rpm
gz topic -i /right_wheel_rpm
```

## Waypoint_Navigation package
A Python based waypoint navigation scripts which navigates by using PID Controller.

### Tuning PID Parameters
To fine-tune the control system, use ```rqt``` for dynamic adjustment of PID Values:
* ```Kp``` (Proportional gain)
* ```Ki``` (Integral gain)
* ```Kd``` (Derivative gain)

### To run Waypoint Navigation
```bash
ros2 run waypoint waypoint_navigation
```
### To check
```bash
ros2 run waypoint waypoint_navigation --ros-args -p waypoint_1_x:=2.0 -p waypoint_1_y:=1.0 -p waypoint_2_x:=4.0 -p waypoint_2_y:=3.0
```
### Terminal Output
```bash
[INFO] [1740153954.085911906] [waypoint_navigation]: Waypoint 1 reached: (2.0, 1.0)
[INFO] [1740153957.661830581] [waypoint_navigation]: Waypoint 2 reached: (4.0, 3.0)
[INFO] [1740153957.663092059] [waypoint_navigation]: Waypoints reached. Stopping robot.
[INFO] [1740153957.664060657] [waypoint_navigation]: Successfully reached all waypoints.
```
