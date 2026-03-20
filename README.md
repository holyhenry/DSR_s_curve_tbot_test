# Human-Led Communication-free Constant-spacing Robot Platoons 
A repo for Turtlebot hardware testing

# Important rospackages
1. camera_multi_tag: launch `image_publisher.py` and `cam_multi_tag_node` to dectect Aruco marker
2. camera_multi_tag_2: launch `cam_multi_tag_2.py` to detect Apriltag
3. tracking: tbot controller 
4. tbot_launch: Main launch file

# Usage
Master device: Either henrypcl (IP:192.168.0.159) or the Alienware laptop (IP: NEED TO CHECK)

## Cmd window 1 - Leader teleoperation:
1. Setup Turtlebot model & Turtlebot namespace

`export ROS_MASTER_URI=...` (if needed)

`export TURTLEBOT3_MODEL=burger` 

`export ROS_NAMESPACE=tbot165`

`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

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
