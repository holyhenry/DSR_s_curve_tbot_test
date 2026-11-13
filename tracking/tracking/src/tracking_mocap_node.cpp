// ROS 1 V formation: copy of tracking_node.cpp with mocap replacing AprilTags.
// Controller equations, LSQ equations, movement gate and loop rate are retained.
// MoCap history is anchored once to the first accepted pose in fresh wheel odometry.
// PoseStamped inputs are base-center poses in one common frame, in meters, +z up.
// A common world rotation/translation cancels when computing the relative pose.
#include <ros/ros.h>
#include "tracking/tracking_mocap_node.hpp"

MocapTrackingNode::MocapTrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                           std::string mode_str, std::string ang_mode_str,
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
    pnh.param<std::string>("formation_type", formation_type_, "v_formation");
    pnh.param<double>("arrow_a", arrow_a_, 0.25);
    pnh.param<double>("arrow_b", arrow_b_, 0.15);
    pnh.param<double>("mocap_timeout", mocap_timeout_, 0.5);
    pnh.param<double>("mocap_max_linear_speed", mocap_max_linear_speed_, 0.0);
    if (!std::isfinite(mocap_max_linear_speed_) || mocap_max_linear_speed_ < 0.0)
        throw std::invalid_argument("mocap_max_linear_speed must be finite and nonnegative");
    pnh.param<int>("mocap_lsq_buffer", mocap_lsq_buffer_, 7);
    if (mocap_lsq_buffer_ < 4 || mocap_lsq_buffer_ > 100)
        throw std::invalid_argument("mocap_lsq_buffer must be between 4 and 100");
    pnh.param<double>("mocap_pair_slop", mocap_pair_slop_, 0.05);
    pnh.param<double>("odom_timeout", odom_timeout_, 0.5);
    pnh.param<bool>("direction_interlock", direction_interlock_, true);
    pnh.param<double>("interlock_los_ahead", interlock_los_ahead_, 0.10);
    pnh.param<double>("interlock_dsr_behind", interlock_dsr_behind_, 0.10);
    if (!std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0 ||
        !std::isfinite(interlock_los_ahead_) || interlock_los_ahead_ <= 0.0 ||
        !std::isfinite(interlock_dsr_behind_) || interlock_dsr_behind_ <= 0.0)
        throw std::invalid_argument("Invalid odom timeout or direction interlock thresholds");
    pnh.param<double>("leader_yaw_offset", leader_yaw_offset_, M_PI);
    pnh.param<double>("follower_yaw_offset", follower_yaw_offset_, 0.0);
    pnh.param<std::string>("mocap_frame", mocap_frame_, "");
    std::string leader_pose_topic, follower_pose_topic;
    pnh.param<std::string>("leader_pose_topic", leader_pose_topic, "/bot205/pose");

    if (formation_type_ != "v_formation")
        throw std::invalid_argument("Mocap tracking requires formation_type v_formation");
    if (follower_indx_ != 0 && follower_indx_ != 1)
        throw std::invalid_argument("V formation requires follower_indx 0 (bot162) or 1 (bot149)");
    if (mode_str != "DSR" || ang_mode_str != "LOS")
        throw std::invalid_argument("V formation requires DSR linear and LOS angular mode");
    if (!std::isfinite(mocap_pair_slop_) || mocap_pair_slop_ <= 0.0 ||
        !std::isfinite(arrow_a_) || !std::isfinite(arrow_b_) ||
        !std::isfinite(mocap_timeout_) || arrow_a_ <= 0.0 ||
        arrow_b_ < 0.0 || mocap_timeout_ <= 0.0 ||
        !std::isfinite(leader_yaw_offset_) || !std::isfinite(follower_yaw_offset_))
        throw std::invalid_argument("Invalid V formation geometry, mocap timeout or heading offsets");
    pnh.param<std::string>("follower_pose_topic", follower_pose_topic,
                          follower_indx_ == 0 ? "/bot162/pose" : "/bot149/pose");
    ROS_INFO("Mocap V formation: follower_indx=%d, leader=%s, follower=%s",
             follower_indx_, leader_pose_topic.c_str(), follower_pose_topic.c_str());
    // Setup publisher (rm "/" in front of topic name to handle ns)
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);

    // Setup logging publishers
    global_leader_pub_ = nh.advertise<geometry_msgs::Pose2D>("global_leader_states", 5);
    controllr_log_pub_ = nh.advertise<std_msgs::Float32MultiArray>("controller_log_info", 5);

    // Wheel odometry still supplies follower motion; mocap supplies relative leader pose.
    odom_sub_ = nh.subscribe("wheelodom", 10, &MocapTrackingNode::odomCallback, this);
    leader_mocap_sub_ = nh.subscribe(leader_pose_topic, 10,
                                    &MocapTrackingNode::leaderMocapCallback, this);
    follower_mocap_sub_ = nh.subscribe(follower_pose_topic, 10,
                                      &MocapTrackingNode::followerMocapCallback, this);

    // Setup controller mode
    if (mode_str == "P")
    {
        mode_ = ControllerMode::P;
    }
    if (mode_str == "DSR")
    {
        mode_ = ControllerMode::DSR;
    }
    if (ang_mode_str == "LOS")
    {
        ang_mode_ = AngControllerMode::LOS;
    }
    if (ang_mode_str == "PC")
    {
        ang_mode_ = AngControllerMode::PC;
    }

    // Initialize odom state
    odom_displacement_ = Eigen::Vector3d::Zero();
    odom_state_last_ = Eigen::Vector3d::Zero();

    // Initialize relative leader measurement (same virtual start trajectory)
    leader_local_state_ = Eigen::Vector2d(spacing_, 0.0);
    if (formation_type_ == "v_formation") {
        // Parallel initial arrangement: leader is forward and opposite the role offset.
        const double side = (follower_indx_ == 0) ? arrow_b_ : -arrow_b_;
        leader_local_state_ = Eigen::Vector2d(arrow_a_, -side);
    }

    // No synthetic local points may be stored as odometry-frame history here.
    // Defer interpTraj until both real relative pose and fresh odometry exist.
    global_leader_states_.resize(0, 2);
    global_leader_state_last_ = Eigen::Vector2d::Zero();

    // Initialize leader observation time stamp
    mocap_t_sec_ = 0.0;

}
// =============================Callback functions=============================

void MocapTrackingNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    const ros::Time& stamp = msg->header.stamp;
    const double age = (ros::Time::now() - stamp).toSec();
    if (stamp.isZero() || age < 0.0 || age > odom_timeout_ ||
        (have_odom_ && stamp <= odom_stamp_)) {
        ROS_WARN_THROTTLE(1.0, "[ODOM reject] zero/future/stale/non-increasing stamp; age=%.3f", age);
        return;
    }
    const auto& pose = msg->pose.pose;
    const auto& orientation = pose.orientation;
    const double qnorm = std::sqrt(orientation.x*orientation.x + orientation.y*orientation.y +
                                   orientation.z*orientation.z + orientation.w*orientation.w);
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(qnorm) || qnorm < 1e-8 || !std::isfinite(msg->twist.twist.linear.x)) {
        ROS_WARN_THROTTLE(1.0, "[ODOM reject] invalid pose/quaternion/velocity");
        return;
    }
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
    q.normalize();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Create odom readings
    Eigen::Vector3d odom_state(x, y, yaw);

    // LSQ filter
    auto res = odomLSQFilter(odom_state, odom_t_sec);
    Eigen::Vector3d odom_state_f  = res.first;
    Eigen::Vector3d disp_over_tau = res.second;

    // Update displacement
    odom_displacement_ = disp_over_tau;
    odom_state_last_   = odom_state_f;
    have_odom_ = true;
    odom_stamp_ = stamp;
    odom_received_ = ros::Time::now();
}

bool MocapTrackingNode::readMocapPose(const geometry_msgs::PoseStamped& msg,
                                     double yaw_offset, Eigen::Vector3d& state) const
{
    // Both poses must describe base centers in the same mocap world frame.
    // Apply each robot's configured rigid-body heading correction.
    const double age = (ros::Time::now() - msg.header.stamp).toSec();
    if (msg.header.stamp.isZero() || age < 0.0 || age > mocap_timeout_ ||
        (!mocap_frame_.empty() && msg.header.frame_id != mocap_frame_))
    {
        ROS_WARN_THROTTLE(1.0, "[MOCAP reject] stamp/frame: age=%.3f frame='%s' required='%s' (check ROS clocks)",
                          age, msg.header.frame_id.c_str(), mocap_frame_.c_str());
        return false;
    }
    const auto& p = msg.pose.position;
    const auto& q = msg.pose.orientation;
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(norm) || norm < 1e-8)
    {
        ROS_WARN_THROTTLE(1.0, "[MOCAP reject] nonfinite position or invalid quaternion");
        return false;
    }
    tf::Quaternion rotation(q.x/norm, q.y/norm, q.z/norm, q.w/norm);
    double roll, pitch, yaw;
    tf::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
    state = Eigen::Vector3d(p.x, p.y, wrapPi(yaw + yaw_offset));
    return state.allFinite();
}

void MocapTrackingNode::leaderMocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (have_leader_mocap_ && msg->header.stamp <= leader_stamp_) {
        ROS_WARN_THROTTLE(1.0, "[MOCAP leader reject] non-increasing timestamp");
        return;
    }
    if (!readMocapPose(*msg, leader_yaw_offset_, leader_mocap_state_)) return;
    leader_stamp_ = msg->header.stamp;
    leader_received_ = ros::Time::now();
    leader_frame_ = msg->header.frame_id;
    have_leader_mocap_ = true;
    new_leader_message_ = true;
}

void MocapTrackingNode::followerMocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (have_follower_mocap_ && msg->header.stamp <= follower_stamp_) {
        ROS_WARN_THROTTLE(1.0, "[MOCAP follower reject] non-increasing timestamp");
        return;
    }
    if (!readMocapPose(*msg, follower_yaw_offset_, follower_mocap_state_)) return;
    follower_stamp_ = msg->header.stamp;
    follower_received_ = ros::Time::now();
    follower_frame_ = msg->header.frame_id;
    have_follower_mocap_ = true;
}

bool MocapTrackingNode::mocapReady() const
{
    if (!have_leader_mocap_ || !have_follower_mocap_ || leader_frame_ != follower_frame_)
        return false;
    const ros::Time now = ros::Time::now();
    const double leader_age = (now - leader_stamp_).toSec();
    const double follower_age = (now - follower_stamp_).toSec();
    const double leader_received_age = (now - leader_received_).toSec();
    const double follower_received_age = (now - follower_received_).toSec();
    // NatNet topics travel through independent UDP streams. Do not combine
    // world poses acquired too far apart: the resulting relative position is
    // not a physically measured robot-to-robot transform.
    const double pair_delta = std::abs((leader_stamp_ - follower_stamp_).toSec());
    return pair_delta <= mocap_pair_slop_ &&
           leader_age >= 0.0 && leader_age <= mocap_timeout_ &&
           follower_age >= 0.0 && follower_age <= mocap_timeout_ &&
           leader_received_age >= 0.0 && leader_received_age <= mocap_timeout_ &&
           follower_received_age >= 0.0 && follower_received_age <= mocap_timeout_;
}

bool MocapTrackingNode::odomReady() const
{
    if (!have_odom_) return false;
    const ros::Time now = ros::Time::now();
    const double age = (now - odom_stamp_).toSec();
    const double received_age = (now - odom_received_).toSec();
    return age >= 0.0 && age <= odom_timeout_ &&
           received_age >= 0.0 && received_age <= odom_timeout_;
}

void MocapTrackingNode::logInputStatus() const
{
    const ros::Time now = ros::Time::now();
    ROS_WARN_THROTTLE(1.0,
        "[INPUT stop] have(L/F/O/accepted/history)=%d/%d/%d/%d/%d ready(mocap/odom)=%d/%d "
        "stamp_age(L/F/O)=%.3f/%.3f/%.3f receive_age=%.3f/%.3f/%.3f "
        "pair_skew=%.3f limit=%.3f odom_leader_skew=%.3f accepted_age=%.3f "
        "frames='%s'/'%s' timeouts(mocap/odom)=%.3f/%.3f",
        have_leader_mocap_, have_follower_mocap_, have_odom_, have_valid_mocap_, history_initialized_,
        mocapReady(), odomReady(),
        (now-leader_stamp_).toSec(), (now-follower_stamp_).toSec(), (now-odom_stamp_).toSec(),
        (now-leader_received_).toSec(), (now-follower_received_).toSec(), (now-odom_received_).toSec(),
        std::abs((leader_stamp_-follower_stamp_).toSec()), mocap_pair_slop_,
        (odom_stamp_-leader_stamp_).toSec(), (now-last_valid_mocap_time_).toSec(),
        leader_frame_.c_str(), follower_frame_.c_str(), mocap_timeout_, odom_timeout_);
}

void MocapTrackingNode::logMotionSigns()
{
    // Compare measured displacements projected on each frame's own forward axis.
    // No frame alignment, sign correction, filtering or control substitution is applied.
    if (have_sign_reference_ && (follower_stamp_-sign_mocap_stamp_).toSec() < 0.5) return;
    if (have_sign_reference_) {
        const Eigen::Vector2d dm = (follower_mocap_state_-sign_mocap_reference_).head<2>();
        const Eigen::Vector2d dw = (odom_state_last_-sign_odom_reference_).head<2>();
        const double mf = dm.dot(Eigen::Vector2d(std::cos(sign_mocap_reference_[2]), std::sin(sign_mocap_reference_[2])));
        const double wf = dw.dot(Eigen::Vector2d(std::cos(sign_odom_reference_[2]), std::sin(sign_odom_reference_[2])));
        ROS_INFO("[SIGN] forward_displacement mocap=%.4f wheel=%.4f m dt(mocap/wheel)=%.3f/%.3f "
                 "wheel_twist_v=%.4f wheel_LSQ_v=%.4f heading_delta(mocap/wheel)=%.3f/%.3f rad",
                 mf, wf, (follower_stamp_-sign_mocap_stamp_).toSec(), (odom_stamp_-sign_odom_stamp_).toSec(),
                 odom_vel_x_, getVelocity(), angDiff(follower_mocap_state_[2], sign_mocap_reference_[2]),
                 angDiff(odom_state_last_[2], sign_odom_reference_[2]));
        if (std::abs(mf) > 0.01 && std::abs(wf) > 0.01 && mf*wf < 0.0)
            ROS_WARN("[SIGN mismatch] wheel and mocap forward motion disagree; inspect heading offsets/odom axes");
    }
    sign_mocap_reference_ = follower_mocap_state_;
    sign_odom_reference_ = odom_state_last_;
    sign_mocap_stamp_ = follower_stamp_;
    sign_odom_stamp_ = odom_stamp_;
    have_sign_reference_ = true;
}

// ===============================Core operations==============================

void MocapTrackingNode::mocapFilter()
{
    const double outlier_threshold = 0.04;
    const double movement_threshold = 0.0004; // Same displacement-over-tau gate as AprilTag.
    if (!new_leader_message_ || !odomReady() || !mocapReady()) return;
    new_leader_message_ = false;

    // Replace camera/tag reconstruction with the mocap world-to-follower transform.
    const double c = std::cos(follower_mocap_state_[2]);
    const double s = std::sin(follower_mocap_state_[2]);
    Eigen::Matrix2d R_inv;
    R_inv << c, s, -s, c;
    const Eigen::Vector2d relative_leader =
        R_inv * (leader_mocap_state_ - follower_mocap_state_).head<2>();
    const double relative_yaw = wrapPi(leader_mocap_state_[2] - follower_mocap_state_[2]);

    // Preserve the original local-measurement outlier rejection and reacquisition.
    const bool stale_reference = have_valid_mocap_ &&
        (ros::Time::now() - last_valid_mocap_time_).toSec() > mocap_timeout_;
    if (have_valid_mocap_ && !stale_reference &&
        (relative_leader - leader_local_state_).norm() > outlier_threshold) {
        ROS_WARN_THROTTLE(0.5, "[MOCAP outlier] jump=%.4f m limit=%.4f accepted_age=%.3f; sample rejected",
                          (relative_leader-leader_local_state_).norm(), outlier_threshold,
                          (ros::Time::now()-last_valid_mocap_time_).toSec());
        return;
    }
    leader_local_state_ = relative_leader;
    leader_relative_yaw_ = relative_yaw;
    have_valid_mocap_ = true;
    last_valid_mocap_time_ = ros::Time::now();
    ROS_INFO_THROTTLE(0.5, "[MOCAP] local leader=(%.3f, %.3f), relative yaw=%.2f deg",
                      leader_local_state_.x(), leader_local_state_.y(),
                      leader_relative_yaw_ * 180.0 / M_PI);
    const Eigen::Vector2d vt = getVFormationTarget();
    ROS_INFO_THROTTLE(0.5, "[V MOCAP] local formation LOS target=(%.3f, %.3f)",
                      vt.x(), vt.y());

    // Transform to global frame
    Eigen::Vector2d global_leader_state = homoTrans2Global(leader_local_state_);

    if (!history_initialized_) {
        // interpTraj returns LOCAL points from -leader to +leader. Transform
        // EVERY point, not only the endpoint, into the current odometry frame.
        controller_.interpTraj(leader_local_state_, 300);
        const Eigen::MatrixXd virtual_local = controller_.getObservations();
        global_leader_states_.resize(virtual_local.rows(), 2);
        for (Eigen::Index i = 0; i < virtual_local.rows(); ++i) {
            const Eigen::Vector2d point = virtual_local.row(i).transpose();
            global_leader_states_.row(i) = homoTrans2Global(point).transpose();
        }
        global_leader_state_last_ = global_leader_state;
        history_initialized_ = true;
        ROS_INFO("[HISTORY init] anchored 300 points to REAL local leader=(%.3f,%.3f) "
                 "at wheel pose=(%.3f,%.3f,%.3f), endpoint_odom=(%.3f,%.3f)",
                 leader_local_state_.x(), leader_local_state_.y(),
                 odom_state_last_[0], odom_state_last_[1], odom_state_last_[2],
                 global_leader_state.x(), global_leader_state.y());
    }
    logMotionSigns();

    // LSQ filter
    mocap_t_sec_ = normalizeMocapTime(leader_stamp_);
    auto res = mocapLSQFilter(global_leader_state, mocap_t_sec_);
    Eigen::Vector2d global_leader_state_f = res.first;
    Eigen::Vector2d disp_over_tau         = res.second;

    // Add new state into system memory (identical to AprilTag).
    const int n = static_cast<int>(global_leader_states_.rows());
    double movement = disp_over_tau.norm();
    memory_movement_ = movement;
    memory_appended_ = false;
    if (movement > movement_threshold)
    {
        constexpr int kMaxStates = 2000;

        if (n < kMaxStates) {
            global_leader_states_.conservativeResize(n + 1, 2);
            global_leader_states_.row(n) = global_leader_state_f.transpose();
        } else {
            // Drop oldest by shifting up 1 row, then write new sample at end
            global_leader_states_.topRows(kMaxStates - 1) =
                global_leader_states_.bottomRows(kMaxStates - 1).eval();

            global_leader_states_.row(kMaxStates - 1) = global_leader_state_f.transpose();
        }
        global_leader_state_last_ = global_leader_state_f;
        memory_appended_ = true;
        ++memory_appends_;
    }

    ROS_INFO_THROTTLE(0.5, "[MEMORY] leader_odom_raw=(%.3f,%.3f) filtered=(%.3f,%.3f) "
                      "disp_tau=%.6f gate=%.6f appended=%d total_appends=%lu samples=%ld endpoint_gap=%.4f",
                      global_leader_state.x(), global_leader_state.y(),
                      global_leader_state_f.x(), global_leader_state_f.y(),
                      memory_movement_, movement_threshold, memory_appended_, memory_appends_,
                      static_cast<long>(global_leader_states_.rows()),
                      (global_leader_state-global_leader_state_last_).norm());

    // Publish the actual last point, including a sample appended this cycle.
    geometry_msgs::Pose2D msg;
    msg.x = global_leader_states_(global_leader_states_.rows() - 1, 0);
    msg.y = global_leader_states_(global_leader_states_.rows() - 1, 1);
    global_leader_pub_.publish(msg);

}


Eigen::Vector2d MocapTrackingNode::getVFormationTarget() const
{
    const double side = (follower_indx_ == 0) ? arrow_b_ : -arrow_b_;
    const Eigen::Vector2d offset(-arrow_a_, side);
    const double c = std::cos(leader_relative_yaw_);
    const double s = std::sin(leader_relative_yaw_);
    Eigen::Matrix2d R;
    R << c, -s, s, c;
    return leader_local_state_ + R * offset;
}

void MocapTrackingNode::runControlStep(bool diagnostic_only)
{
    if (!odomReady() || !mocapReady() || !have_valid_mocap_ || !history_initialized_ ||
        (ros::Time::now() - last_valid_mocap_time_).toSec() < 0.0 ||
        (ros::Time::now() - last_valid_mocap_time_).toSec() > mocap_timeout_) {
        // Diagnostic mode publishes no cmd_vel at all, including stop messages.
        if (!diagnostic_only) cmd_vel_pub_.publish(geometry_msgs::Twist());
        logInputStatus();
        return;
    }
    // 0. Get node current time
    controller_.setNodeTime(normalizeNodeTime(ros::Time::now()));

    // 1. Compute predecessor states in current local frame
    Eigen::MatrixXd local_leader_states = homoInvTransMulti2Local(getGlobalLeaderStates());

    // 2. Feed predecessor trajectory into the controller's buffer
    controller_.setObservations(local_leader_states, mocap_t_sec_);

    // 3. Compute the tracking target point
    bool MEMORY_MODE = true;
    Eigen::Vector2d target = controller_.getTarget(MEMORY_MODE);
    Eigen::Vector2d angular_target = target;
    if (formation_type_ == "v_formation") {
        angular_target = getVFormationTarget();
        ROS_INFO_THROTTLE(1.0, "[V] LOS target=(%.3f, %.3f), linear target=(%.3f, %.3f)",
                          angular_target.x(), angular_target.y(), target.x(), target.y());
    }

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
    double angular_controller;
    switch (ang_mode_)
    {
        case AngControllerMode::LOS:
            angular_controller = controller_.angularUpdate(angular_target);

            break;
        case AngControllerMode::PC:
            const double v_current = getVelocity();
            angular_controller = controller_.trajAngularUpdate(v_current);
            break;
    }
    std::vector<double> cmd = controller_.step(linear_controller,
                                               angular_controller);

    // 5. Publish velocity command & debug info
    geometry_msgs::Twist twist_msg;
    // Cap only the output command; retain the calculated controller value for diagnostics.
    double speed_limited_linear = cmd[0];
    if (mocap_max_linear_speed_ > 0.0 && std::isfinite(speed_limited_linear)) {
        if (speed_limited_linear > mocap_max_linear_speed_)
            speed_limited_linear = mocap_max_linear_speed_;
        else if (speed_limited_linear < -mocap_max_linear_speed_)
            speed_limited_linear = -mocap_max_linear_speed_;
    }
    twist_msg.linear.x = speed_limited_linear;
    twist_msg.angular.z = cmd[1];

    const bool direction_conflict = direction_interlock_ &&
        angular_target.x() > interlock_los_ahead_ && target.x() < -interlock_dsr_behind_;
    const bool invalid_command = !target.allFinite() || !angular_target.allFinite() ||
        !std::isfinite(cmd[0]) || !std::isfinite(cmd[1]);
    if (direction_conflict || invalid_command) {
        twist_msg = geometry_msgs::Twist();
        ROS_WARN_THROTTLE(0.5, "[INTERLOCK stop] conflict=%d invalid=%d LOS_x=%.3f DSR_x=%.3f "
                          "calculated_cmd=(%.3f,%.3f); target unchanged; diagnostic_only=%d",
                          direction_conflict, invalid_command, angular_target.x(), target.x(),
                          cmd[0], cmd[1], diagnostic_only);
    }
    const Eigen::Index last = global_leader_states_.rows()-1;
    const Eigen::Vector2d history_local = local_leader_states.row(last).transpose();
    ROS_INFO_THROTTLE(0.5, "[CONTROL] diagnostic_only=%d mocap_leader_world=(%.3f,%.3f,%.3f) "
                      "leader_local=(%.3f,%.3f) wheel_pose=(%.3f,%.3f,%.3f) "
                      "memory_last_odom=(%.3f,%.3f) memory_last_local=(%.3f,%.3f) samples=%ld "
                      "DSR_target=(%.3f,%.3f) LOS_target=(%.3f,%.3f) "
                      "calculated_cmd=(%.4f,%.4f) speed_limited_linear=%.4f "
                      "guarded_cmd=(%.4f,%.4f) blocked=%d",
                      diagnostic_only, leader_mocap_state_[0], leader_mocap_state_[1], leader_mocap_state_[2],
                      leader_local_state_.x(), leader_local_state_.y(),
                      odom_state_last_[0], odom_state_last_[1], odom_state_last_[2],
                      global_leader_states_(last,0), global_leader_states_(last,1),
                      history_local.x(), history_local.y(), static_cast<long>(global_leader_states_.rows()),
                      target.x(), target.y(), angular_target.x(), angular_target.y(),
                      cmd[0], cmd[1], speed_limited_linear, twist_msg.linear.x, twist_msg.angular.z,
                      direction_conflict || invalid_command);
    if (!diagnostic_only) cmd_vel_pub_.publish(twist_msg);

    ControllerDebug dbg = controller_.getDebugData();
    Eigen::Vector3d odom_state = odom_state_last_;
    Eigen::Vector3d odom_disp = odom_displacement_;

    std_msgs::Float32MultiArray msg;
    msg.data.push_back(dbg.error);            // 0  -
    msg.data.push_back(dbg.target_last[0]);   // 1  - x of target_last
    msg.data.push_back(dbg.target_last[1]);   // 2  - y of target_last
    msg.data.push_back(dbg.t_d_last);         // 3  -
    msg.data.push_back(dbg.s_d_last);         // 4  -
    msg.data.push_back(dbg.e_d_last);         // 5  -
    msg.data.push_back(dbg.r_t_last);         // 6  -
    msg.data.push_back(leader_relative_yaw_);             // 7  - relative leader heading
    msg.data.push_back(odom_state[0]);        // 8  - odom x
    msg.data.push_back(odom_state[1]);        // 9  - odom y
    msg.data.push_back(odom_state[2]);        // 10 - odom yaw
    msg.data.push_back(odom_disp[0]);         // 11 - dx
    msg.data.push_back(odom_disp[1]);         // 12 - dy
    msg.data.push_back(odom_disp[2]);         // 13 - dyaw
    controllr_log_pub_.publish(msg);
}

// ==============================Helper functions==============================

std::pair<Eigen::Vector3d, Eigen::Vector3d>
MocapTrackingNode::odomLSQFilter(const Eigen::Vector3d& y_now, const double t_now)
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
MocapTrackingNode::mocapLSQFilter(const Eigen::Vector2d& y_now, const double t_now){

    // Add current sample
    mocap_lsq_t_.push_back(t_now);
    mocap_lsq_y_.push_back(y_now);

    // Enforce window size
    while (static_cast<int>(mocap_lsq_t_.size()) > mocap_lsq_buffer_){
        mocap_lsq_t_.pop_front();
        mocap_lsq_y_.pop_front();
    }
    const size_t m = mocap_lsq_t_.size();
    if (m < 4) return { y_now, Eigen::Vector2d::Zero() };

    // Build LSQ matrix
    Eigen::MatrixXd A(m, 2);
    Eigen::MatrixXd Y(m, 2);
    for (size_t i = 0; i < m; ++i){
        A(i, 0) = mocap_lsq_t_[i];
        A(i, 1) = 1.0;
        Y.row(i) = mocap_lsq_y_[i].transpose();
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

double MocapTrackingNode::normalizeNodeTime(const ros::Time& t)
{
    if (!have_node_time0_) {
        have_node_time0_ = true;
        node_time0_ = t.toSec();
    }
    return t.toSec() - node_time0_;
}

double MocapTrackingNode::normalizeMocapTime(const ros::Time& t)
{
    if (!have_mocap_time0_) {
        have_mocap_time0_ = true;
        mocap_time0_ = t.toSec();
    }
    return t.toSec() - mocap_time0_;
}

double MocapTrackingNode::normalizeOdomTime(const ros::Time& t)
{
    if (!have_odom_time0_) {
        have_odom_time0_ = true;
        odom_time0_ = t.toSec();
    }
    return t.toSec() - odom_time0_;
}

Eigen::Vector2d MocapTrackingNode::homoTrans2Global(const Eigen::Vector2d& local_point)
{
    double theta = odom_state_last_[2];
    Eigen::Matrix2d R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);

    Eigen::Vector2d t(odom_state_last_[0], odom_state_last_[1]);
    return R * local_point + t;
}

Eigen::MatrixXd MocapTrackingNode::homoInvTransMulti2Local(const Eigen::MatrixXd& global_points)
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

Eigen::MatrixXd MocapTrackingNode::getGlobalLeaderStates() const { return global_leader_states_; }

double MocapTrackingNode::getVelocity() const
{
    // Velocity from odmetry can be noisy.
    // This is instead created from smoothed odometry global displacement

    // Planar global velocity dot heading unit vector gives signed forward speed
    const Eigen::Vector2d v_xy = odom_displacement_.head<2>() / tau_;
    const double yaw = odom_state_last_[2];
    const Eigen::Vector2d forward(std::cos(yaw), std::sin(yaw));

    return v_xy.dot(forward);
}

// ==================================Main loop=================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_mocap_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Rate rate(25.0);

    // Platoon parameters
    bool diagnostic_only = true;
    pnh.param<bool>("diagnostic_only", diagnostic_only, true);
    std::string ctrl_mode;
    std::string ang_ctrl_mode;
    double alpha, alpha_angle, beta1, beta2, tau, spacing;
    int follower_indx;

    // Load ROS parameters
    pnh.param<std::string>("ctrl_mode", ctrl_mode, "DSR");
    pnh.param<std::string>("ang_ctrl_mode", ang_ctrl_mode, "LOS");
    pnh.param<double>("alpha", alpha, 0.3);
    pnh.param<double>("alpha_angle", alpha_angle, 3.0);
    pnh.param<double>("beta1", beta1, 0.8);
    pnh.param<double>("beta2", beta2, 0.94);
    pnh.param<double>("tau", tau, 0.04);
    pnh.param<double>("spacing", spacing, 0.25);
    pnh.param<int>("follower_indx", follower_indx, 0);

    // Call the node constructor
    MocapTrackingNode node(nh, pnh,
                      ctrl_mode, ang_ctrl_mode,
                      alpha, alpha_angle,
                      beta1, beta2,
                      tau, spacing,
                      follower_indx);

    ROS_INFO("Mocap tracking node is running...");

    while (ros::ok())
    {
        // Process incoming messages (wheel odometry and both mocap poses)
        ros::spinOnce();

        // Run the relative leader measurement filtering
        node.mocapFilter();

        // Run controller step
        node.runControlStep(diagnostic_only);

        // Sleep to maintain loop rate
        rate.sleep();
    }
    return 0;
}
