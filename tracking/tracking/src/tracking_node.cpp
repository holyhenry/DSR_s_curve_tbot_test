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
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);

    // Setup logging publishers 
    global_leader_pub_ = nh.advertise<geometry_msgs::Pose2D>("global_leader_states", 5);
    controllr_log_pub_ = nh.advertise<std_msgs::Float32MultiArray>("controller_log_info", 5);

    // Setup scuscribers (rm "/" in front of topic name to handle ns)
    odom_sub_ = nh.subscribe("wheelodom", 10, &TrackingNode::odomCallback, this);
    tag_sub_ = nh.subscribe("tag_detections", 10, &TrackingNode::tagCallback, this);

    // Setup controller mode
    if (mode_str == "P") 
    {   
        mode_ = ControllerMode::P;
    }
    if (mode_str == "DSR") 
    {  
        mode_ = ControllerMode::DSR;
    }

    // Initialize odom state 
    odom_displacement_ = Eigen::Vector3d::Zero();
    odom_state_last_ = Eigen::Vector3d::Zero();

    // Initialize apriltag local measurement 
    tag_leader_state_ = Eigen::Vector2d(spacing_, 0.0);

    // Initialize global leader state
    global_leader_state_last_ = Eigen::Vector2d(spacing_, 0.0);
    global_leader_states_ = Eigen::MatrixXd(1, 2);
    global_leader_states_.row(0) << spacing_, 0.0;
    cam_t_secs_.push_back(0.0);

}
// =============================Callback functions=============================

void TrackingNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    const ros::Time& stamp = msg->header.stamp;
    double odom_t_sec = normalizeOdomTime(stamp);

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
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Create odom readings
    Eigen::Vector3d odom_state(x, y, yaw);
    
    // LSQ filter 
    auto res = odomLSQFilter(odom_state, odom_t_sec);
    Eigen::Vector3d odom_state_f  = res.first;
    Eigen::Vector3d disp_over_tau = res.second;

    // Update displacement
    // odom_displacement_ = odom_state - odom_state_last_;
    // odom_state_last_   = odom_state;
    odom_displacement_ = disp_over_tau;
    odom_state_last_   = odom_state_f;
}

void TrackingNode::tagCallback(const common_msgs::Float32ArrayStamped::ConstPtr& msg)
{   
    // Data format: {id, tag_x, tag_y, tag_z, tag_yaw}
    tag_stamp_     = msg->header.stamp;
    normalizeCamTime(tag_stamp_);
    tag_multi_raw_ = msg->data.data;
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
    const double movement_threshold = 0.0004; // min speed 0.01 m/s * 0.04 delay = 0.0004

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
            int id = static_cast<int>(tag_multi_raw_[i]);
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

        // LSQ filter 
        double cam_t_sec = normalizeCamTime(tag_stamp_);
        auto res = camLSQFilter(global_leader_state, cam_t_sec);
        Eigen::Vector2d global_leader_state_f = res.first;
        Eigen::Vector2d disp_over_tau         = res.second;

        // [TODO]: old code below, delete later
        // double movement = (global_leader_state - global_leader_state_last_).norm();
        double movement = disp_over_tau.norm();
        
        if (movement > movement_threshold)
        {   
            ROS_INFO_STREAM("YAS!!!!!!!!! (movement > threshold)");
            cam_t_secs_.push_back(cam_t_sec);
            global_leader_states_.conservativeResize(global_leader_states_.rows() + 1, 2);
            global_leader_states_.row(global_leader_states_.rows() - 1) = global_leader_state_f.transpose();
            global_leader_state_last_ = global_leader_state_f;
            // [TODO]: old code below, delete later
            // global_leader_states_.row(global_leader_states_.rows() - 1) = global_leader_state.transpose();
            // global_leader_state_last_ = global_leader_state;
        }

        // Publish global leader states
        const int last = global_leader_states_.rows() - 1;
        geometry_msgs::Pose2D msg;
        msg.x = global_leader_states_(last, 0);  // coordinate x
        msg.y = global_leader_states_(last, 1);  // coordinate y
        global_leader_pub_.publish(msg);

        // Log current tag position
        ROS_INFO_STREAM("cam_t_sec " << cam_t_sec);
        // ROS_INFO_STREAM("movement " << movement);
        // ROS_INFO_STREAM("tag_leader_state_: " << tag_leader_state_[0] << ", " << tag_leader_state_[1] << ")");
        // ROS_INFO_STREAM("global_leader_state: " << global_leader_state[0] << ", " << global_leader_state[1] << ")");
        ROS_INFO_STREAM("global_leader_state_f: " << global_leader_state_f[0] << ", " << global_leader_state_f[1] << ")");
        ROS_INFO_STREAM("global_leader_states_ size: " << global_leader_states_.size());
    }
}

void TrackingNode::runControlStep()
{   
    // 0. Get node current time 
    controller_.setNodeTime(normalizeNodeTime(ros::Time::now()));

    // 1. Compute predecessor states in current local frame
    Eigen::MatrixXd local_leader_states = homoInvTransMulti2Local(getGlobalLeaderStates());        
    // ROS_INFO_STREAM_THROTTLE(1.0, "local_leader_states:\n" << local_leader_states);

    // 2. Feed predecessor trajectory into the controller's buffer
    controller_.setObservations(local_leader_states, cam_t_secs_);

    // 3. Compute the tracking target point
    bool MEMORY_MODE = true;
    auto target_info = controller_.getTarget(MEMORY_MODE);
    Eigen::Vector2d target = target_info.first;
    double target_t = target_info.second;

    // 4. Run control logic 
    double linear_controller;
    switch (mode_)
    {
        case ControllerMode::P:
            linear_controller = controller_.PUpdate(target);
            break;
        case ControllerMode::DSR:
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

std::pair<Eigen::Vector3d, Eigen::Vector3d> 
TrackingNode::odomLSQFilter(const Eigen::Vector3d& y_now, const double t_now)
{   
    // Unwarp yaw (-pi, pi] to continuous euler value for LSQ
    const double yaw_raw = y_now[2];
    if (!have_last_yaw_){
        have_last_yaw_ = true;
        yaw_unwrap_ = yaw_raw;
        last_yaw_raw_ = yaw_raw;
    } else {
        yaw_unwrap_ += angDiff(yaw_raw, last_yaw_raw_);
        last_yaw_raw_ = yaw_raw;
    }
    Eigen::Vector3d y_now_unwrap(y_now[0], y_now[1], yaw_unwrap_);

    // Add current sample
    lsq_t_.push_back(t_now);
    lsq_y_.push_back(y_now_unwrap);

    // Enforce window size
    while (static_cast<int>(lsq_t_.size()) > lsq_buffer_){
        lsq_t_.pop_front();
        lsq_y_.pop_front();
    }
    const size_t m = lsq_t_.size();
    if (m < 4) return { y_now, Eigen::Vector3d::Zero() };

    // Build LSQ matrix
    Eigen::MatrixXd A(m, 2);
    Eigen::MatrixXd Y(m, 3);
    for (size_t i = 0; i < m; ++i){
        A(i, 0) = lsq_t_[i];
        A(i, 1) = 1.0;
        Y.row(i) = lsq_y_[i].transpose();
    }

    // Solve A*x = Y (x is 2x3: slopes & intercepts for x,y,yaw)
    Eigen::Matrix<double, 2, 3> x = A.colPivHouseholderQr().solve(Y);

    // Evaluate at t_now & t_now - tau_
    Eigen::Vector2d a(t_now, 1.0);
    Eigen::Vector2d a_delay(t_now - tau_, 1.0);

    // Make prediction at t_now & t_now - tau_
    Eigen::Vector3d y_now_f_unwrap = x.transpose() * a;  // (3x2) * (2x1) = 3x1
    Eigen::Vector3d y_delay_unwrap = x.transpose() * a_delay;

    Eigen::Vector3d disp = y_now_f_unwrap - y_delay_unwrap;

    // Wrap yaw outputs back to (-pi, pi]
    Eigen::Vector3d y_now_f = y_now_f_unwrap;
    y_now_f[2] = wrapPi(y_now_f[2]);
    disp[2] = wrapPi(disp[2]);

    return { y_now_f, disp };
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> 
TrackingNode::camLSQFilter(const Eigen::Vector2d& y_now, const double t_now){

    // Add current sample
    cam_lsq_t_.push_back(t_now);
    cam_lsq_y_.push_back(y_now);

    // Enforce window size
    while (static_cast<int>(cam_lsq_t_.size()) > cam_lsq_buffer_){
        cam_lsq_t_.pop_front();
        cam_lsq_y_.pop_front();
    }
    const size_t m = cam_lsq_t_.size();
    if (m < 4) return { y_now, Eigen::Vector2d::Zero() };

    // Build LSQ matrix 
    Eigen::MatrixXd A(m, 2);
    Eigen::MatrixXd Y(m, 2);
    for (size_t i = 0; i < m; ++i){
        A(i, 0) = cam_lsq_t_[i];
        A(i, 1) = 1.0;
        Y.row(i) = cam_lsq_y_[i].transpose();
    }

    // Solve A*x = Y (x is 2x2: slopes & intercepts for x,y,yaw)
    Eigen::Matrix<double, 2, 2> x = A.colPivHouseholderQr().solve(Y);

    // Evaluate at t_now & t_now - tau_
    Eigen::Vector2d a(t_now, 1.0);
    Eigen::Vector2d a_delay(t_now - tau_, 1.0);

    // Make prediction at t_now & t_now - tau_
    Eigen::Vector2d y_now_f = x.transpose() * a;  // (2x2) * (2x1) = 2x1
    Eigen::Vector2d y_delay = x.transpose() * a_delay;

    Eigen::Vector2d disp = y_now_f - y_delay;

    return { y_now_f, disp };
}

double TrackingNode::normalizeNodeTime(const ros::Time& t) 
{
    if (!have_node_time0_) {
        have_node_time0_ = true;
        node_time0_ = t.toSec();
    }
    return t.toSec() - node_time0_;
}

double TrackingNode::normalizeCamTime(const ros::Time& t)
{
    if (!have_tag_time0_) {
        have_tag_time0_ = true;
        tag_time0_ = t.toSec();
    }
    return t.toSec() - tag_time0_;
}

double TrackingNode::normalizeOdomTime(const ros::Time& t)
{
    if (!have_odom_time0_) {
        have_odom_time0_ = true;
        odom_time0_ = t.toSec();
    }
    return t.toSec() - odom_time0_;
}

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
    std::string controller_mode;
    double alpha, alpha_angle, beta1, beta2, tau, spacing;
    int follower_indx;

    // Load ROS parameters
    pnh.param<std::string>("mode", controller_mode, "DSR");
    pnh.param<double>("alpha", alpha, 0.3);
    pnh.param<double>("alpha_angle", alpha_angle, 3.0);
    pnh.param<double>("beta1", beta1, 0.8);
    pnh.param<double>("beta2", beta2, 0.94);
    pnh.param<double>("tau", tau, 0.04);
    pnh.param<double>("spacing", spacing, 0.25);
    pnh.param<int>("follower_indx", follower_indx, 0);

    // Call the node constructor
    TrackingNode node(nh, pnh, controller_mode, alpha, alpha_angle, beta1, beta2, tau, spacing, follower_indx);

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
