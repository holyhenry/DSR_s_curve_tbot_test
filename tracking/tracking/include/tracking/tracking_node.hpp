#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include "controller.hpp"

class TrackingNode{
public:
    TrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void run();

private:
    // ROS interface 
    ros::Publisher cmd_vel_pub_;

    ros::Subscriber odom_sub_;
    ros::Subscriber tag_sub_;

    // Core timer
    ros::Rate loop_rate_;
    double dt_;

    // Controller object
    Controller controller_;

    // Robot & tag states
    std::vector<double> current_state_;  // [x, y, yaw]
    std::vector<double> tag_state_;      // [x, y, yaw]
    double velocity_;

    // Parameters
    double spacing_, alpha_, beta_1, beta_2;

    // Callbacks
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void tagCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
};