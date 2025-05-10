#include "tracking/controller.hpp"

Controller::Controller(double alpha,
                       double alpha_angle,
                       double beta1,
                       double beta2,
                       double tau,
                       double spacing)
    : alpha_(alpha), alpha_angle_(alpha_angle),
      beta1_(beta1), beta2_(beta2), tau_(tau), spacing_(spacing),
      error_(0.0), t_d_last_(0.0), s_d_last_(0.0), r_t_last_(0.0)
{   
    // System memories
    error_ = 0.0;                             // controller logitudinal error
    observations_ = Eigen::MatrixXd(0, 2);    // local predecessor trajectory
    input_        = Eigen::Vector2d::Zero();  // controller command
    target_last_  = Eigen::Vector2d::Zero(); 
}

// ==============================Core functions==============================

double Controller::angularUpdate(const Eigen::Vector2d& target) const
{
    double theta = std::atan2(target[1], target[0]);
    double angle_fb = alpha_angle_ * theta;

    return checkLimits(angle_fb, omega_min_, omega_max_);
}

double Controller::PUpdate(const Eigen::Vector2d& target) 
{
    error_ = signedError(target);
    double linear_fb = alpha_ * error_;

    return checkLimits(linear_fb, vel_min_, vel_max_);
}

double Controller::DSRUpdate(const Eigen::Vector2d& target, const Eigen::Vector2d& displacement) 
{   
    error_ = signedError(target);
    double delta_target = signedError(target - target_last_);
    double delta_state = signedError(displacement); 

    // Low-pass derivative terms
    const double lp_gain = 0.167;
    double t_d = lowPass(delta_target / tau_, t_d_last_, lp_gain);
    double s_d = lowPass(delta_state / tau_, s_d_last_, lp_gain);

    // Compute reinforce term
    double reinforce = beta1_ * t_d  + (beta2_ - beta1_) * s_d; 
    double r_t = lowPass(reinforce, r_t_last_, lp_gain);

    double linear_fb = alpha_ * beta1_ * error_ + r_t;

    // Update controller memories
    target_last_ = target;
    t_d_last_ = t_d;
    s_d_last_ = s_d;
    r_t_last_ = r_t;

    return checkLimits(linear_fb, vel_min_, vel_max_);
}

Eigen::Vector2d Controller::getTarget(bool memory_mode)
{
    if (observations_.rows() == 0)  
    {
        ROS_WARN("[getTarget]: No observations.");
        
        return Eigen::Vector2d::Zero();
    }

    Eigen::Vector2d leader_current_state = observations_.row(observations_.rows() - 1);
    if (memory_mode)
    {
        // Compute distance from each row in observations_ to current leader
        Eigen::MatrixXd diffs = observations_.rowwise() - leader_current_state.transpose();
        Eigen::VectorXd dists = diffs.rowwise().norm();

        // Find valid indices where dist >= spacing_
        std::vector<int> valid_indicies;
        for (int i = 0; i < dists.size(); ++i)
        {
            if (dists[i] >= spacing_)
                valid_indicies.push_back(i);
        }

        // Return the valid index that is cloest to the bot
        if (!valid_indicies.empty())
        {
            int indx = valid_indicies.back();
            Eigen::RowVectorXd target = observations_.row(indx);  // (1 × 2) row
            
            return target;
        }
    }

    // Fallback: non memory_mode or no valid target found
    ROS_INFO_STREAM_THROTTLE(1.0, "[getTarget]: MEMORIZED TARGET NOT FOUND. Using fallback.");

    Eigen::Vector2d direction = - leader_current_state;
    double theta = std::atan2(direction[1], direction[0]);
    Eigen::Vector2d fallback = leader_current_state + spacing_ * Eigen::Vector2d(std::cos(theta), std::sin(theta));
    
    return fallback;
}

void Controller::setObservations(const Eigen::MatrixXd& observations)
{
    observations_ = observations;
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

// ==========================Debug Helper functions==========================

ControllerDebug Controller::getDebugData() const
{
    return { target_last_, t_d_last_, s_d_last_, r_t_last_ };
}