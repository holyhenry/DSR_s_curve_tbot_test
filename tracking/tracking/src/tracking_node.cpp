#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_node");
    ros::NodeHandle nh;
    ROS_INFO("Dummy tracking node is running...");
    ros::spin();
    return 0;
}
