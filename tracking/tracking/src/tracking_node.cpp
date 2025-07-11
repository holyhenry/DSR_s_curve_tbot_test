#include <ros/ros.h>
#include "tracking/tracking_node.hpp"

TrackingNode::TrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string mode_str,
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

    // Setup logging publishers 
    global_leader_pub_ = nh.advertise<std_msgs::Float32MultiArray>("global_leader_states", 1);
    controllr_log_pub_ = nh.advertise<std_msgs::Float32MultiArray>("controller_log_info", 1);


    // Setup scuscribers (rm "/" in front of topic name to handle ns)
    odom_sub_ = nh.subscribe("wheelodom", 5, &TrackingNode::odomCallback, this);
    tag_sub_ = nh.subscribe("tag_detections", 5, &TrackingNode::tagCallback, this);

    // Setup controller mode
    if (mode_str == "P") mode_ = ControllerMode::P;
    if (mode_str == "DSR") mode_ = ControllerMode::DSR;

    // Initialize odom state 
    odom_displacement_ = Eigen::Vector3d::Zero();
    odom_state_last_ = Eigen::Vector3d::Zero();

    // Initialize apriltag local measurement 
    tag_leader_state_ = Eigen::Vector2d(spacing_, 0.0);

    // Initialize global leader state
    global_leader_state_last_ = Eigen::Vector2d(spacing_, 0.0);
    global_leader_states_ = Eigen::MatrixXd(0, 2);

}
// =============================Callback functions=============================

void TrackingNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Extract pose & velocity
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    odom_vel_x_ = msg->twist.twist.linear.x;

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
    Eigen::Vector3d odom_state(x, y, yaw);
    odom_displacement_ = odom_state - odom_state_last_;
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
    const int tag_space = 5;
    const int id_offset = 100;
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
        bool isCorrectID = static_cast<int>(tag_multi_raw_[i] - id_offset) / num_tag == follower_indx_;
        if (i % tag_space == 0 && isCorrectID)
        {   
            int id = static_cast<int>(tag_multi_raw_[i] - id_offset);
            double x   =   tag_multi_raw_[i + 3];
            double y   = -(tag_multi_raw_[i + 1] - cam_pos_offset);
            double phi =  -tag_multi_raw_[i + 4];

            Eigen::Vector3d infered = transformTag2Middle(x, y, phi, id);
            Eigen::Vector2d inferred_bot = homoTrans2BotCenter(infered);

            // Filter the outliers
            double norm_diff = (inferred_bot - tag_leader_state_).norm();
            bool isOutlier = norm_diff > outlier_threshold;

            if (!isOutlier)
            {
                count++;
                tag_x += infered[0];
                tag_y += infered[1];
                tag_phi += infered[2];
            }
            else
            {   // Log outlier detection
                ROS_INFO_STREAM("Filtered tag ID:" << id);
            }
        }
    }

    if (count > 0)
    {
        tag_x /= count;
        tag_y /= count;
        tag_phi /= count;

        Eigen::Vector3d tag_raw(tag_x, tag_y, tag_phi);
        tag_leader_state_ = homoTrans2BotCenter(tag_raw);

        // Transform to global frame
        Eigen::Vector2d global_leader_state = homoTrans2Global(tag_leader_state_);
        double movement = (global_leader_state - global_leader_state_last_).norm();

        if (movement > movement_threshold)
        {
            global_leader_states_.conservativeResize(global_leader_states_.rows() + 1, 2);
            global_leader_states_.row(global_leader_states_.rows() - 1) = global_leader_state.transpose();
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
        // ROS_INFO_STREAM_THROTTLE(2.0, "global_leader_states_ size: " << global_leader_states_.rows());
    }
}

void TrackingNode::runControlStep()
{   
    // 1. Compute predecessor states in current local frame
    Eigen::MatrixXd local_leader_states = homoInvTransMulti2Local(getGlobalLeaderStates());        
    // ROS_INFO_STREAM_THROTTLE(1.0, "local_leader_states:\n" << local_leader_states);

    // 2. Feed predecessor trajectory into the controller's buffer
    controller_.setObservations(local_leader_states);

    // 3. Compute the tracking target point
    bool MEMORY_MODE = true;
    Eigen::Vector2d target = controller_.getTarget(MEMORY_MODE);

    // 4. Run control logic 
    double linear_controller;
    switch (mode_)
    {
        case ControllerMode::P:
            linear_controller = controller_.PUpdate(target);
            break;
        case ControllerMode::DSR:
            ROS_INFO_STREAM(" D S R in use");
            linear_controller = controller_.DSRUpdate(target, odom_displacement_.head<2>());
            break;
    }
    double angular_controller = controller_.angularUpdate(target); // or TrajAngularUpdate()
    std::vector<double> cmd = controller_.step(linear_controller,
                                               angular_controller);

    // 5. Publish velocity command & debug info
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = cmd[0];
    twist_msg.angular.z = cmd[1];
    cmd_vel_pub_.publish(twist_msg);

    ControllerDebug dbg = controller_.getDebugData();
    Eigen::Vector3d odom_state = odom_state_last_;
    Eigen::Vector3d odom_disp = odom_displacement_;

    std_msgs::Float32MultiArray msg;
    msg.data.push_back(dbg.error);
    msg.data.push_back(dbg.target_last[0]);   // x of target_last
    msg.data.push_back(dbg.target_last[1]);   // y of target_last
    msg.data.push_back(dbg.t_d_last);
    msg.data.push_back(dbg.s_d_last);
    msg.data.push_back(dbg.reinforce_term);
    msg.data.push_back(odom_vel_x_);
    msg.data.push_back(odom_state[0]);        // odom x
    msg.data.push_back(odom_state[1]);        // odom y
    msg.data.push_back(odom_state[2]);        // odom yaw
    msg.data.push_back(odom_disp[0]);         // dx
    msg.data.push_back(odom_disp[1]);         // dy
    msg.data.push_back(odom_disp[2]);         // dyaw
    controllr_log_pub_.publish(msg);
}

// ==============================Helper functions==============================

Eigen::Vector3d TrackingNode::transformTag2Middle(double x, double y, double alpha,
                                                      int id, int num_tag, double d)
{   // frame size 4.0cm, d=0.038mm
    // frame size 4.5cm, d=0.043mm
    // frame size 5.0cm, d=0.048mm
    if (id % num_tag == 0)
    {
        x += d * std::cos(-M_PI / 2 + alpha + 0.2616);
        y += d * std::sin(-M_PI / 2 + alpha + 0.2616);
        alpha += 30.0 * M_PI / 180.0;
    }
    else if (id % num_tag == 2)
    {
        x += d * std::cos(M_PI / 2 + alpha - 0.2616);
        y += d * std::sin(M_PI / 2 + alpha - 0.2616);
        alpha -= 30.0 * M_PI / 180.0;
    }
    return Eigen::Vector3d(x, y, alpha);
}

Eigen::Vector2d TrackingNode::homoTrans2BotCenter(const Eigen::Vector3d& state)
{
    double theta = state[2];
    Eigen::Matrix2d R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);

    Eigen::Vector2d t = state.head<2>();
    Eigen::Matrix3d T;
    T << R, t,
         0, 0, 1;

    Eigen::Vector3d tag_offset(0.14, 0.0, 1.0);  // Offset from tag to robot center
    Eigen::Vector3d transformed = T * tag_offset;

    return transformed.head<2>();
}

Eigen::Vector2d TrackingNode::homoTrans2Global(const Eigen::Vector2d& local_point)
{
    double theta = odom_state_last_[2];
    Eigen::Matrix2d R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);

    Eigen::Vector2d t(odom_state_last_[0], odom_state_last_[1]);
    return R * local_point + t;
}

Eigen::MatrixXd TrackingNode::homoInvTransMulti2Local(const Eigen::MatrixXd& global_points)
{
    double theta = odom_state_last_[2];
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    // Rotation matrix transpose (inverse rotation)
    Eigen::Matrix2d R_inv;
    R_inv << cos_theta, sin_theta,
            -sin_theta, cos_theta;

    Eigen::Vector2d t(odom_state_last_[0], odom_state_last_[1]);

    // Shift all global points by -t, then rotate into local frame
    Eigen::MatrixXd shifted = global_points.rowwise() - t.transpose();
    Eigen::MatrixXd local_points = (R_inv * shifted.transpose()).transpose();

    return local_points;
}

Eigen::MatrixXd TrackingNode::getGlobalLeaderStates() const { return global_leader_states_; }

// ==================================Main loop=================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Rate rate(25.0);

    // Platoon parameters
    std::string mode_str;
    double alpha, alpha_angle, beta1, beta2, tau, spacing;
    int follower_indx;

    // Load ROS parameters
    pnh.param<std::string>("controller_mode", mode_str, "DSR");
    pnh.param<double>("alpha", alpha, 0.3);
    pnh.param<double>("alpha_angle", alpha_angle, 3.0);
    pnh.param<double>("beta1", beta1, 0.8);
    pnh.param<double>("beta2", beta2, 0.94);
    pnh.param<double>("tau", tau, 0.02);
    pnh.param<double>("spacing", spacing, 0.25);
    pnh.param<int>("follower_indx", follower_indx, 0);

    // Call the node constructor
    TrackingNode node(nh, pnh, mode_str, alpha, alpha_angle, beta1, beta2, tau, spacing, follower_indx);

    ROS_INFO("Tracking node is running...");

    while (ros::ok())
    {
        // Process incoming messages (odom, tag detections)
        ros::spinOnce();

        // Run the tag filtering
        node.aprilTagFilter();

        // Run controller step
        node.runControlStep();

        // Sleep to maintain loop rate
        rate.sleep();
    }
    return 0;
}
