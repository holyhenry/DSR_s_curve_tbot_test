#include "tracking/controller.hpp"

Controller::Controller(double alpha,
                       double alpha_angle,
                       double beta1,
                       double beta2,
                       double tau,
                       double spacing)
    : alpha_(alpha), alpha_angle_(alpha_angle),
      beta1_(beta1), beta2_(beta2), tau_(tau), spacing_(spacing),
      error_last_(0.0), t_d_last_(0.0), s_d_last_(0.0), e_d_last_(0.0), r_t_last_(0.0)
{   
    // System memories
    error_last_ = 0.0;                        // controller logitudinal error
    observations_ = Eigen::MatrixXd(0, 2);    // local predecessor trajectory
    input_        = Eigen::Vector2d::Zero();  // controller command
    target_last_  = Eigen::Vector2d::Zero();
}

// ==============================Core functions==============================

double Controller::angularUpdate(const Eigen::Vector2d& target) const
{
    double theta = std::atan2(target[1], target[0]);

    // Stop rotating if the angular error is too large (±60 deg)
    if (std::abs(theta) > M_PI / 3.0)
    {
        return 0.0;
    }

    double angle_fb = alpha_angle_ * theta;
    return checkLimits(angle_fb, omega_min_, omega_max_);
}

double Controller::PUpdate(const Eigen::Vector2d& target) 
{
    double error = signedError(target);
    double linear_fb = alpha_ * error;

    double error_last_ = error;

    return checkLimits(linear_fb, vel_min_, vel_max_);
}

double Controller::DSRUpdate(const Eigen::Vector2d& target, const Eigen::Vector2d& displacement) 
{   
    double error = signedError(target);
    double delta_target = (target - target_last_).norm();
    double delta_state = displacement.norm(); 

    // Low-pass derivative terms
    const double lp_gain = 1.0;

    // Implementation 1 
    double t_d = lowPass(delta_target / tau_, t_d_last_, lp_gain);
    // double s_d = lowPass(delta_state / tau_, s_d_last_, lp_gain);
    // double reinforce = beta1_ * t_d  + (beta2_ - beta1_) * s_d; 

    // Implementation 2
    auto res = errorLSQFilter(error, obs_t_sec_);
    double error_f = res.first;
    double delta_error = res.second;
    // double delta_error = error - error_last_;
    double e_d = lowPass(delta_error / tau_, e_d_last_, lp_gain);
    double s_d = lowPass(delta_state / tau_, s_d_last_, lp_gain);
    double reinforce = beta1_ * e_d  + beta2_ * s_d; 
    
    double r_t = lowPass(reinforce, r_t_last_, 1.0);
    double linear_fb = alpha_ * error_f + r_t;

    // Update controller memories
    target_last_ = target;
    error_last_ = error_f;
    t_d_last_ = t_d;
    s_d_last_ = s_d;
    e_d_last_ = e_d;
    r_t_last_ = r_t;

    return checkLimits(linear_fb, vel_min_, vel_max_);
}

Eigen::Vector2d Controller::getTarget(bool memory_mode)
{
    if (observations_.rows() == 0){
        ROS_WARN("[getTarget]: No observations.");
        
        return Eigen::Vector2d::Zero();
    }

    Eigen::Vector2d leader_current_state = observations_.row(observations_.rows() - 1);
    if (memory_mode){
        // Compute distance from each row in observations_ to current leader
        Eigen::MatrixXd diffs = observations_.rowwise() - leader_current_state.transpose();
        Eigen::VectorXd dists = diffs.rowwise().norm();

        // Find valid indices where dist >= spacing_
        std::vector<int> valid_indicies;
        for (int i = 0; i < dists.size(); ++i){
            if (dists[i] >= spacing_)
                valid_indicies.push_back(i);
        }

        // Return the valid index that is cloest to the bot
        if (!valid_indicies.empty()){
            int indx = valid_indicies.back();
            Eigen::RowVectorXd target = observations_.row(indx);  // (1 × 2) row
            
            return target;
        }
    }

    // Fallback: non memory_mode or no valid target found
    ROS_WARN("[getTarget]: MEMORIZED TARGET NOT FOUND. Using fallback.");

    Eigen::Vector2d direction = - leader_current_state;
    double theta = std::atan2(direction[1], direction[0]);
    Eigen::Vector2d fallback = leader_current_state + spacing_ * Eigen::Vector2d(std::cos(theta), std::sin(theta));

    return fallback;
}

void Controller::setNodeTime(double node_t_sec)
{
    node_t_sec_ = node_t_sec;
}

void Controller::setObservations(const Eigen::MatrixXd& observations,
                                 double cam_t_sec)
{
    observations_ = observations;
    obs_t_sec_ = cam_t_sec;
}

std::vector<double> Controller::step(double linear_update, double anguler_update)
{
    input_ << linear_update, anguler_update;  // Update robot controller input

    return toStdVector(input_);
}

// =============================Helper functions=============================

double Controller::lowPass(double x, double x_last, double gain) const 
{
    return gain * x + (1.0 - gain) * x_last;
}

double Controller::checkLimits(double u, double min, double max) const 
{
    return std::max(min, std::min(max, u));
}

double Controller::signedError(const Eigen::Vector2d& target) const 
{
    // Robot is at origin (0, 0) in local frame
    // Heading is fixed: +x
    // So the target is either in front (x > 0) or behind (x < 0)
    double magnitude = std::sqrt(target[0]*target[0] + target[1]*target[1]);
    double sign = (target[0] >= 0.0) ? 1.0 : -1.0;

    return magnitude * sign;
}

std::vector<double> Controller::toStdVector(const Eigen::Vector2d& v) const
{
    return std::vector<double>(v.data(), v.data() + v.size());
}

std::pair<double, double> 
Controller::errorLSQFilter(const double y_now, const double t_now){

    // Add current sample
    err_lsq_t_.push_back(t_now);
    err_lsq_y_.push_back(y_now);

    // Enforce window size
    while (static_cast<int>(err_lsq_t_.size()) > err_lsq_buffer_){
        err_lsq_t_.pop_front();
        err_lsq_y_.pop_front();
    }
    const size_t m = err_lsq_t_.size();
    if (m < 4) return { y_now, 0.0 };

    // Build LSQ system: A * x = Y, where
    // A = [ t_i  1 ]
    // x = [ slope, intercept ]^T
    // Y = [ y_i ]
    Eigen::MatrixXd A(m, 2);
    Eigen::VectorXd Y(m);
    for (size_t i = 0; i < m; ++i){
        A(i, 0) = err_lsq_t_[i];
        A(i, 1) = 1.0;
        Y(i) = err_lsq_y_[i];
    }

    // Solve A*x = Y (x is 2x1: slope & intercept)
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(Y);

    // Evaluate at t_now & t_now - tau_
    Eigen::Vector2d a(t_now, 1.0);
    Eigen::Vector2d a_delay(t_now - tau_, 1.0);

    // Make prediction at t_now & t_now - tau_
    const double y_now_f = x.dot(a);        // x^T * a
    const double y_delay = x.dot(a_delay);  // x^T * a_delay

    const double disp = y_now_f - y_delay;

    return { y_now_f, disp };
}

// ==========================Debug Helper functions==========================

ControllerDebug Controller::getDebugData() const
{
    return { error_last_, target_last_, t_d_last_, s_d_last_, e_d_last_, r_t_last_ };
}
