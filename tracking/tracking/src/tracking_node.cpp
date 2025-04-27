#include <ros/ros.h>
#include "tracking/tracking_node.hpp"

TrackingNode::TrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                           double alpha, double alpha_angle,
                           double beta1, double beta2,
                           double tau, double spacing)
    : loop_rate_(20.0),
      controller_(alpha, alpha_angle, beta1, beta2, tau, spacing),
      alpha_(alpha), alpha_angle_(alpha_angle),
      beta1_(beta1), beta2_(beta2),
      tau_(tau), spacing_(spacing)
{
    // Setup publishers
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 2);

    // Setup scuscribers
    odom_sub_ = nh.subscribe("/odom", 5, &TrackingNode::odomCallback, this);
    tag_sub_ = nh.subscribe("/tag_detections", 5, &TrackingNode::tagCallback, this);

    // Initialize global state 
    displacement_ = {0.0, 0.0 ,0.0};
    odom_state_last_ = {0.0, 0.0 ,0.0};

    // Initialize local measurement
    tag_state_ = {spacing_, 0.0};

}

void TrackingNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Extract pose
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Extract yaw from quaternion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Update displacement
    std::vector<double> odom_state = {x, y, yaw};
    displacement_[0] = odom_state[0] - odom_state_last_[0];
    displacement_[1] = odom_state[1] - odom_state_last_[1];
    displacement_[2] = odom_state[2] - odom_state_last_[2];

    // Update last state
    odom_state_last_ = odom_state;
}

void TrackingNode::tagCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // TODO: Process tag measurement
    ROS_INFO("TODO: Process tag measurement");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double alpha, alpha_angle, beta1, beta2, tau, spacing;

    // Load ROS params manually here
    pnh.param<double>("alpha", alpha, 0.3);
    pnh.param<double>("alpha_angle", alpha_angle, 20.0);
    pnh.param<double>("beta1", beta1, 0.8);
    pnh.param<double>("beta2", beta2, 0.95);
    pnh.param<double>("tau", tau, 0.02);
    pnh.param<double>("spacing", spacing, 0.25);

    // Call the constructor
    TrackingNode node(nh, pnh, alpha, alpha_angle, beta1, beta2, tau, spacing);

    ROS_INFO("Dummy tracking node is running...");
    ros::spin();
    return 0;
}
