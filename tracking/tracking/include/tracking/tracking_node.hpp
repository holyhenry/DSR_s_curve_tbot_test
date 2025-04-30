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
                 double tau, double spacing,
                 int follower_indx);

    // =============================Core functions=============================
    void aprilTagFilter();
    void run();

private:
    // Platoon parameters
    double alpha_, alpha_angle_, beta1_, beta2_;
    double spacing_, tau_;
    int follower_indx_;

    // Controller object
    Controller controller_;

    // Robot odom states (global frame)
    std::vector<double> odom_displacement_; // [x, y, yaw]
    std::vector<double> odom_state_last_;   // [x, y, yaw]

    // Robot tag raw & filtered detection (local frame)
    std::vector<float> tag_multi_raw_;           // raw apriltag detection
    std::vector<double> tag_leader_state_;       // filtered apriltag detection  [x, y]

    // ROS interface & timer 
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber tag_sub_;
    ros::Rate loop_rate_;

    // =============================Internal functions=============================

    // Callback functions
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void tagCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

    // Helper functions
    std::vector<double> transformTag2Middle(double x, double y, double alpha, 
                                            int id, int num_tag = 3, double d = 0.043);
    std::vector<double> addVectors(const std::vector<double>& a,
                                   const std::vector<double>& b);
    std::vector<double> homoTrans2D(const std::vector<double>& robot_pose,
                                    const std::vector<double>& point);
    std::vector<double> homoTrans2BotCenter(const std::vector<double>& state);
    std::vector<double> homoTrans2Global(const std::vector<double>& local_point);

};