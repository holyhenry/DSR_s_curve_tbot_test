# DSR_s_curve_tbot_test
A repo for Turtlebot hardware testing

# Usage
Master device: Either henrypcl (IP:192.168.0.159) or the Alienware laptop (IP: NEED TO CHECK)

## Cmd window 1 - Leader teleoperation:
1. Setup Turtlebot model & Turtlebot namespace

`export ROS_MASTER_URI=...` (if needed)

`export TURTLEBOT3_MODEL=burger` 

`export ROS_NAMESPACE=tbot165`

## Cmd window 2 - Bringup and open camera:
2. Bringup all Turtlebots
3. Launch the camera and Apriltag detection nodes

`roslaunch tbot_launch tbot_launch_mock.launch`

In the launch file:
- If the master PC is henrypcl use line 5: `<arg name="env" default="/opt/ros/noetic/env_henry.sh"/>`
- If the master PC is Alienware use line 6: `<arg name="env" default="/opt/ros/noetic/env_alien.sh"/>`

## Cmd window 3 - Run DSR or baseline controller:
4. Launch controller  

`roslaunch pd_tracking_xy pd_tracking_xy.launch`
