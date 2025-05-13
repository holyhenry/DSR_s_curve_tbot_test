#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class OdomCovarianceInjector
{
public:
    OdomCovarianceInjector(ros::NodeHandle& nh)
    {
        odom_sub_ = nh.subscribe("odom", 10, &OdomCovarianceInjector::odomCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("wheelodom", 10);
    }

private:
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        nav_msgs::Odometry odom = *msg;  // Copy message

        // Set realistic pose covariance (x, y, yaw)
        odom.pose.covariance[0] = 0.05;    // x 0.01
        odom.pose.covariance[7] = 0.05;    // y 0.01
        odom.pose.covariance[14] = 1.0e-9; // z (unused)
        odom.pose.covariance[21] = 1.0e-9; // roll
        odom.pose.covariance[28] = 1.0e-9; // pitch
        odom.pose.covariance[35] = 0.0872; // yaw

        // Optional: twist covariance (linear + angular velocity)
        odom.twist.covariance[0] = 0.01;
        odom.twist.covariance[7] = 0.01;
        odom.twist.covariance[35] = 0.02;

        odom_pub_.publish(odom);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_cov_injector");
    ros::NodeHandle nh;
    OdomCovarianceInjector wrapper(nh);
    ros::spin();
    return 0;
}
