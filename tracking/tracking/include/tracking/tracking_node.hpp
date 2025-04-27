#pragma once

#include <tf/tf.h>  // for quaternion to yaw conversion
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include "controller.hpp"

class TrackingNode{
public:
    TrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                 double alpha, double alpha_angle,
                 double beta1, double beta2,
                 double tau, double spacing);

    void run();

private:
    // Parameters
    double alpha_, alpha_angle_, beta1_, beta2_;
    double spacing_, tau_;

    // ROS interface 
    ros::Publisher cmd_vel_pub_;

    ros::Subscriber odom_sub_;
    ros::Subscriber tag_sub_;

    // Core timer
    ros::Rate loop_rate_;

    // Controller object
    Controller controller_;

    // Robot & tag states
    std::vector<double> displacement_;     // [x, y, yaw]
    std::vector<double> odom_state_last_;  // [x, y, yaw]
    std::vector<double> tag_state_;        // [x, y]

    // Callbacks
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void tagCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
};