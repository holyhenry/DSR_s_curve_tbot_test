#include <ros/ros.h>
#include "tracking/tracking_node.hpp"

TrackingNode::TrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                           double alpha, double alpha_angle,
                           double beta1, double beta2,
                           double tau, double spacing,
                           int follower_indx)
    : loop_rate_(20.0),
      controller_(alpha, alpha_angle, beta1, beta2, tau, spacing),
      alpha_(alpha), alpha_angle_(alpha_angle),
      beta1_(beta1), beta2_(beta2),
      tau_(tau), spacing_(spacing),
      follower_indx_(follower_indx)
{
    // Setup publisher (rm "/" in front of topic name to handle ns)
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 2);
    // Setup debug publishers 
    global_leader_pub_ = nh.advertise<std_msgs::Float32MultiArray>("global_leader_states", 1);

    // Setup scuscribers (rm "/" in front of topic name to handle ns)
    odom_sub_ = nh.subscribe("odom", 5, &TrackingNode::odomCallback, this);
    tag_sub_ = nh.subscribe("tag_detections", 5, &TrackingNode::tagCallback, this);

    // Initialize odom state 
    odom_displacement_ = {0.0, 0.0 ,0.0};
    odom_state_last_ = {0.0, 0.0 ,0.0};

    // Initialize apriltag local measurement 
    tag_leader_state_ = {spacing_, 0.0};

    // Initialize global leader state
    global_leader_state_last_ = {spacing_, 0.0};
    global_leader_states_ = Eigen::MatrixXd(0, 2);

}
// =============================Callback functions=============================

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
    odom_displacement_[0] = odom_state[0] - odom_state_last_[0];
    odom_displacement_[1] = odom_state[1] - odom_state_last_[1];
    odom_displacement_[2] = odom_state[2] - odom_state_last_[2];

    // Update last state
    odom_state_last_ = odom_state;
}

void TrackingNode::tagCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{   
    // Data format: {id, tag_x, tag_y, tag_z, tag_yaw}
    tag_multi_raw_ = msg->data;
}

// ===============================Core operations==============================

void TrackingNode::aprilTagFilter()
{
    int count = 0;
    const int num_tag = 3;
    const int raw_tag_space = 5;
    const double cam_pos_offset = 0.025;
    const double outlier_threshold = 0.04;
    const double movement_threshold = 0.005;

    double tag_x = 0.0;
    double tag_y = 0.0;
    double tag_phi = 0.0;

    if (tag_multi_raw_.empty())
    {
        ROS_WARN_THROTTLE(2.0, "No Apriltag detected.");
        return;
    }

    for (size_t i = 0; i < tag_multi_raw_.size(); ++i)
    {   
        bool isCorrectID = static_cast<int>(tag_multi_raw_[i]) / num_tag == follower_indx_;
        if (i % raw_tag_space == 0 && isCorrectID)
        {   
            int id = static_cast<int>(tag_multi_raw_[i]);
            double x   =   tag_multi_raw_[i + 3];
            double y   = -(tag_multi_raw_[i + 1] - cam_pos_offset);
            double phi =  -tag_multi_raw_[i + 4];

            std::vector<double> infered = transformTag2Middle(x, y, phi, id);
            double infered_x = infered[0];
            double infered_y = infered[1];
            double infered_phi = infered[2];

            // Filter the outliers
            std::vector<double> infered_bot = homoTrans2BotCenter(infered);
            std::vector<double> tag_diff = {infered_bot[0] - tag_leader_state_[0],
                                            infered_bot[1] - tag_leader_state_[1]};
            double norm_diff = std::sqrt(tag_diff[0]*tag_diff[0] + tag_diff[1]*tag_diff[1]);
            bool isOutlier = norm_diff > outlier_threshold;

            if (!isOutlier)
            {
                count++;
                tag_x += infered_x;
                tag_y += infered_y;
                tag_phi += infered_phi;
            }
            else
            {   // Log outlier detection
                // ROS_INFO_STREAM("Filtered tag ID:" << id);
            }
        }
    }

    if (count > 0)
    {
        tag_x /= count;
        tag_y /= count;
        tag_phi /= count;

        std::vector<double> tag_raw = {tag_x, tag_y, tag_phi};
        tag_leader_state_ = homoTrans2BotCenter(tag_raw);

        // Transform to global frame
        std::vector<double> global_leader_state = homoTrans2Global(tag_leader_state_);
        double dx = global_leader_state[0] - global_leader_state_last_[0];
        double dy = global_leader_state[1] - global_leader_state_last_[1];
        double movement = std::sqrt(dx * dx + dy * dy);

        if (movement > movement_threshold)
        {
            global_leader_states_.conservativeResize(global_leader_states_.rows() + 1, 2);
            global_leader_states_.row(global_leader_states_.rows() - 1) << global_leader_state[0], global_leader_state[1];
            global_leader_state_last_ = global_leader_state;
        }

        // Publish global leader states
        std_msgs::Float32MultiArray msg;
        for (int i = 0; i < global_leader_states_.rows(); ++i)
        {
            msg.data.push_back(global_leader_states_(i, 0));
            msg.data.push_back(global_leader_states_(i, 1));
        }
        global_leader_pub_.publish(msg);

        // Log current tag position
        // ROS_INFO_STREAM("tag_leader_state_: " << tag_leader_state_[0] << ", " << tag_leader_state_[1] << ")");
        ROS_INFO_STREAM("global_leader_states_ size: " << global_leader_states_.rows());
    }
}

// ==============================Helper functions==============================

std::vector<double> TrackingNode::transformTag2Middle(double x, double y, double alpha,
                                                      int id, int num_tag, double d)
{   // frame size 4.0cm, d=0.038mm
    // frame size 4.5cm, d=0.043mm
    // frame size 5.0cm, d=0.048mm
    if (id % num_tag == 0)
    {
        double x1_hat = x + d * std::cos(-M_PI/2.0 + alpha + 0.2616);
        double y1_hat = y + d * std::sin(-M_PI/2.0 + alpha + 0.2616);
        alpha += (30.0 / 180.0) * M_PI;
        return {x1_hat, y1_hat, alpha};
    }

    if (id % num_tag == 2)
    {
        double x1_hat = x + d * std::cos(M_PI/2.0 + alpha - 0.2616);
        double y1_hat = y + d * std::sin(M_PI/2.0 + alpha - 0.2616);
        alpha -= (30.0 / 180.0) * M_PI;
        return {x1_hat, y1_hat, alpha};
    }

    return {x, y, alpha};
}

std::vector<double> TrackingNode::addVectors(const std::vector<double>& a,
                                             const std::vector<double>& b)
{
    if (a.size() != b.size())
    {
        ROS_ERROR("Vector size mismatch in addVectors!");
        return {};  // Return empty vector if sizes don't match
    }

    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i)
    {
        result[i] = a[i] + b[i];
    }
    return result;
}

std::vector<double> TrackingNode::homoTrans2D(const std::vector<double>& robot_pose,
                                              const std::vector<double>& point)
{
    double theta = robot_pose[2];
    double x = robot_pose[0];
    double y = robot_pose[1];

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    double new_x = cos_theta * point[0] - sin_theta * point[1] + x;
    double new_y = sin_theta * point[0] + cos_theta * point[1] + y;

    return {new_x, new_y};
}

std::vector<double> TrackingNode::homoTrans2BotCenter(const std::vector<double>& state)
{   
    // Tag center to robot center offset
    std::vector<double> offset = {0.14, 0.0}; 
    return homoTrans2D(state, offset);
}

std::vector<double> TrackingNode::homoTrans2Global(const std::vector<double>& local_point)
{   
    // current_state_ holds robot's global [x, y, yaw]
    std::vector<double> current_state_ = addVectors(odom_state_last_, odom_displacement_);
    return homoTrans2D(current_state_, local_point); 
}

// ==================================Main loop=================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Rate rate(20.0);

    // Platoon parameters
    double alpha, alpha_angle, beta1, beta2, tau, spacing;
    int follower_indx;

    // Load ROS parameters
    pnh.param<double>("alpha", alpha, 0.3);
    pnh.param<double>("alpha_angle", alpha_angle, 20.0);
    pnh.param<double>("beta1", beta1, 0.8);
    pnh.param<double>("beta2", beta2, 0.95);
    pnh.param<double>("tau", tau, 0.02);
    pnh.param<double>("spacing", spacing, 0.35);
    pnh.param<int>("follower_indx", follower_indx, 0);

    // Call the node constructor
    TrackingNode node(nh, pnh, alpha, alpha_angle, beta1, beta2, tau, spacing, follower_indx);

    ROS_INFO("Tracking node is running...");

    while (ros::ok())
    {
        // Process incoming messages (odom, tag detections, etc.)
        ros::spinOnce();

        // Run the leader tag filtering
        node.aprilTagFilter();

        // Later you can add node.controlStep();  // For actual controller output

        // Sleep to maintain loop rate
        rate.sleep();
    }
    return 0;
}
