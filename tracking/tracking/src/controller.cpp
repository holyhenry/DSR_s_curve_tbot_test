#include "tracking/controller.hpp"

Controller::Controller(double alpha,
                       double alpha_angle,
                       double beta1,
                       double beta2,
                       double tau,
                       double spacing)
    : alpha_(alpha), alpha_angle_(alpha_angle),
      beta1_(beta1), beta2_(beta2), tau_(tau), spacing_(spacing),
      error_(0.0), t_d_last_(0.0), e_l_last_(0.0)
{   
    // System memories
    error_ = 0.0;  // logitudinal error
    observations_ = Eigen::MatrixXd(0, 2);          // predecessor observations
    input_        = std::vector<double>{0.0, 0.0};  // controller command
    target_last_  = std::vector<double>{0.0, 0.0}; 
    state_last_   = std::vector<double>{0.0, 0.0};
    
}
// ==============================Core functions==============================

double Controller::PUpdate(const std::vector<double>& target) 
{
    error_ = signedError(target);
    double linear_fb = alpha_ * error_;

    return checkLimits(linear_fb);
}

double Controller::DSRUpdate(const std::vector<double>& target) 
{
    error_ = signedError(target);
    // TODO: finish DSR update 2

    return 0.0;
}

std::vector<double> Controller::getTarget(bool memory_mode)
{
    if (observations_.rows() == 0)  
    {
        ROS_WARN("[getTarget]: No observations.");
        
        return std::vector<double>{0.0, 0.0};
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
            
            return std::vector<double>{target[0], target[1]};
        }
    }
    
    // Fallback: non memory_mode or no valid target found
    ROS_WARN("[getTarget]: MEMORIZED TARGET NOT FOUND. Using fallback.");

    Eigen::Vector2d direction = - leader_current_state;
    double theta = std::atan2(direction[1], direction[0]);
    Eigen::Vector2d fallback = leader_current_state + spacing_ * Eigen::Vector2d(std::cos(theta), std::sin(theta));
    
    return std::vector<double>{fallback[0], fallback[1]};
}

// =============================Helper functions=============================

double Controller::lowPass(double x, double x_last, double gain) const 
{
    return gain * x + (1.0 - gain) * x_last;
}

double Controller::checkLimits(double u) const 
{
    return std::max(min_, std::min(max_, u));
}

double Controller::signedError(const std::vector<double>& target) const 
{
    // Robot is at origin (0, 0) in local frame
    // Heading is fixed: +x
    // So the target is either in front (x > 0) or behind (x < 0)
    double magnitude = std::sqrt(target[0]*target[0] + target[1]*target[1]);
    double sign = (target[0] >= 0.0) ? 1.0 : -1.0;

    return magnitude * sign;
}