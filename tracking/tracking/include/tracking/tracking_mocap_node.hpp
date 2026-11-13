#pragma once

#include <deque>
#include <stdexcept>
#include <string>
#include <utility>
#include <cmath>
#include <tf/tf.h>  // for quaternion to yaw conversion
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include "controller.hpp"

class MocapTrackingNode
{
public:
    MocapTrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                 std::string mode_str, std::string ang_mode_str,
                 double alpha, double alpha_angle,
                 double beta1, double beta2,
                 double tau, double spacing,
                 int follower_indx);

    // P - Proportional, DSR - Delayed Self-Reinforcement
    enum class ControllerMode { P, DSR };
    ControllerMode mode_;
    // LOS - Line of Sight, PC - Path Curvature
    enum class AngControllerMode { LOS, PC };
    AngControllerMode ang_mode_;

    // =============================Core functions=============================
    Eigen::MatrixXd getGlobalLeaderStates() const;
    void mocapFilter();
    void runControlStep();

    // =============================Getter functions=============================
    double getVelocity() const;

private:
    // Formation parameters
    double alpha_, alpha_angle_, beta1_, beta2_;
    double spacing_, tau_;
    int follower_indx_;
    std::string formation_type_ = "v_formation";
    double arrow_a_ = 0.25, arrow_b_ = 0.15;
    double mocap_timeout_ = 0.5;
    double mocap_pair_slop_ = 0.05;     // max difference between leader/follower stamps
    double mocap_memory_step_ = 0.01;  // min real filtered movement to append memory
    bool have_valid_mocap_ = false;
    ros::Time last_valid_mocap_time_;

    // Controller object
    Controller controller_;

    // Robot odom states (GLOBAL frame) & velocity (LOCAL frame)
    Eigen::Vector3d odom_displacement_; // [x, y, yaw]
    Eigen::Vector3d odom_state_last_;   // [x, y, yaw]
    double odom_vel_x_ = 0.0;           // vy ≈ 0 for diff-drive

    // Mocap is used only to replace the relative leader measurement.
    Eigen::Vector2d leader_local_state_;
    double leader_relative_yaw_ = 0.0;
    Eigen::Vector3d leader_mocap_state_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d follower_mocap_state_ = Eigen::Vector3d::Zero();
    double leader_yaw_offset_ = M_PI, follower_yaw_offset_ = M_PI;
    std::string mocap_frame_, leader_frame_, follower_frame_;
    ros::Time leader_stamp_, follower_stamp_;
    ros::Time leader_received_, follower_received_;
    bool new_leader_message_ = false;
    bool have_leader_mocap_ = false, have_follower_mocap_ = false;
    bool have_odom_ = false;

    // Leader trajectory in the follower odometry frame
    Eigen::Vector2d global_leader_state_last_; // [x, y]
    Eigen::MatrixXd global_leader_states_;     // Nx2 matrix
    double mocap_t_sec_;                         // leader mocap timestamp

    // ROS interface & timer
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber leader_mocap_sub_;
    ros::Subscriber follower_mocap_sub_;
    ros::Rate loop_rate_;
    // ROS interface (debugging & visualizing)
    ros::Publisher global_leader_pub_;
    ros::Publisher controllr_log_pub_;

    // Odom LSQ filter state (3D)
    int lsq_buffer_ = 10;
    std::deque<double>          lsq_t_;
    std::deque<Eigen::Vector3d> lsq_y_;

    // Leader measurement LSQ filter state (2D)
    int mocap_lsq_buffer_ = 7;
    std::deque<double>           mocap_lsq_t_;
    std::deque<Eigen::Vector2d>  mocap_lsq_y_;

    // Odom yaw unwrapping state
    bool have_last_yaw_  = false;
    double last_yaw_raw_ = 0.0;
    double yaw_unwrap_   = 0.0;

    // Time normalization anchors
    bool have_node_time0_ = false;
    double node_time0_    = 0.0;
    bool have_mocap_time0_  = false;
    double mocap_time0_     = 0.0;
    bool have_odom_time0_ = false;
    double odom_time0_    = 0.0;

    // =============================Internal functions=============================

    // Callback functions
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void leaderMocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void followerMocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool readMocapPose(const geometry_msgs::PoseStamped& msg, double yaw_offset,
                       Eigen::Vector3d& state) const;
    bool mocapReady() const;

    // Static helper functions
    static double wrapPi(double a){
        a = std::fmod(a + M_PI, 2.0*M_PI);
        if (a < 0) a += 2.0*M_PI;
        return a - M_PI;
    }
    static double angDiff(double a, double b){
        return wrapPi(a - b);
    }

    // Helper functions
    Eigen::Vector2d getVFormationTarget() const;
    double normalizeNodeTime(const ros::Time& t);
    double normalizeMocapTime(const ros::Time& t);
    double normalizeOdomTime(const ros::Time& t);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> odomLSQFilter(const Eigen::Vector3d& y_now,
                                                                  const double t_now);
    std::pair<Eigen::Vector2d, Eigen::Vector2d> mocapLSQFilter(const Eigen::Vector2d& y_now,
                                                                  const double t_now);
    Eigen::Vector2d homoTrans2Global(const Eigen::Vector2d& local_point);
    Eigen::MatrixXd homoInvTransMulti2Local(const Eigen::MatrixXd& global_points);
};
