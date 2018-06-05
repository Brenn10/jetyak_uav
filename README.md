# jetyak_uav
This package is being developed by the Autonomous Field Robotics Lab as an interface between a GPS-enabled ground vehicle and a multicopter. This package contains two ROS-catkin packages: uav_maneuver and jetyak_uav_utils. uav_maneuver contains maneuvers that will interface with the uav away from the jetyak and the jetyak_uav_utils package contains utilities for while the uav is either cooperating with the jetyak or attempting to return.

Current design is for DJI N3 controller with a raspberry pi interface. Must set Lightbridge Controller to P_MODE before control will be able to be requested by the package. The package will ***NOT*** immediately request control, the X button on the controller must be used to request control.

## Usage
call ```roslaunch jetyak_uav_utils full.launch```
Launches dji_sdk, ar_track_alvar, raspicam_node, and all jetyak_uav_utils nodes


## jetyak_uav_utils Nodes
### controller_node
#### Subsciptions
* joy (sensor_msgs::Joy) for use with logitech gamepad
  * RT, LT - Deadswitch
  * B - Give up control
  * A - Land
  * Y - Takeoff
  * X - Take back control
  * Right stick up - move forward
  * Right stick right - move right
  * Left stick up - increase altitude
  * Left stick right - turn right
* uav_mode (jetyak_uav_utils::Mode)
  * Header header
  * int8 mode
  * LANDED=0
  * TAKINGOFF=1
  * FOLLOWING=2
  * AWAY=3
  * SEARCHING=4
  * APPROACHING=5
  * LANDING=6
* raw_cmd (geometry_msgs::Twist)
  * linear.x : Forward
  * linear.y : Left
  * linear.z : Up
  * angular.z : Counter-Clockwise
#### Publications
* /dji_sdk/flight_control_setpoint_generic (sensor_msgs::Joy)
  * (linear.x, linear.y, linear.z, angular.z, 0x4B)
  * 0x4B = Command Horizontal Velocities + Command Vertical Speed + Command Yaw speed + Active break
* uav_mode (jetyak_uav_utils::Mode)
  * Publishes when gamepad takes off or lands (AWAY and LANDED respectively)

### take_off_follow_node
#### Subscriptions
#### Publications
