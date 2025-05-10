#pragma once

#include <tf/tf.h>  // for quaternion to yaw conversion
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include "controller.hpp"

class TrackingNode
{
public:
    TrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string mode_str,
                 double alpha, double alpha_angle,
                 double beta1, double beta2,
                 double tau, double spacing,
                 int follower_indx);
    
    enum class ControllerMode { P, DSR };
    ControllerMode mode_;

    // =============================Core functions=============================
    Eigen::MatrixXd getGlobalLeaderStates() const;
    void aprilTagFilter();
    void runControlStep();

private:
    // Platoon parameters
    double alpha_, alpha_angle_, beta1_, beta2_;
    double spacing_, tau_;
    int follower_indx_;

    // Controller object
    Controller controller_;

    // Robot odom states (GLOBAL frame) & velocity (LOCAL frame)
    Eigen::Vector3d odom_displacement_; // [x, y, yaw]
    Eigen::Vector3d odom_state_last_;   // [x, y, yaw]
    double odom_vel_x_ = 0.0;           // vy ≈ 0 for diff-drive

    // Robot tag raw & filtered detection (LOCAL frame)
    std::vector<float> tag_multi_raw_;  // raw apriltag detection
    Eigen::Vector2d tag_leader_state_;  // [x, y] - filtered apriltag detection
    
    // Robot tag transformed global leader states (GLOBAL frame)
    Eigen::Vector2d global_leader_state_last_; // [x, y]
    Eigen::MatrixXd global_leader_states_;     // Nx2 matrix

    // ROS interface & timer 
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber tag_sub_;
    ros::Rate loop_rate_;
    // ROS interface (debugging & visualizing)
    ros::Publisher global_leader_pub_;
    ros::Publisher controllr_log_pub_;

    // =============================Internal functions=============================

    // Callback functions
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void tagCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

    // Helper functions
    Eigen::Vector3d transformTag2Middle(double x, double y, double alpha, 
                                        int id, int num_tag = 3, double d = 0.043);
    Eigen::Vector2d homoTrans2BotCenter(const Eigen::Vector3d& state);
    Eigen::Vector2d homoTrans2Global(const Eigen::Vector2d& local_point);
    Eigen::MatrixXd homoInvTransMulti2Local(const Eigen::MatrixXd& global_points);
};