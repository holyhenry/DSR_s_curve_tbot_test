#include "tracking/controller.hpp"
#include <numeric>
#include <cmath>
#include <algorithm>

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
    input_       = std::vector<double>{0.0, 0.0};
    target_last_ = std::vector<double>{0.0, 0.0};
    state_last_  = std::vector<double>{0.0, 0.0};

    // Logitudinal parameters
    error_ = 0.0;
}

double Controller::lowPass(double x, double x_last, double gain) const {
    return gain * x + (1.0 - gain) * x_last;
}

double Controller::checkLimits(double u) const {
    return std::max(min_, std::min(max_, u));
}

double Controller::signedError(const std::vector<double>& target) const {
    // Robot is at origin (0, 0) in local frame
    // Heading is fixed: +x
    // So the target is either in front (x > 0) or behind (x < 0)
    double magnitude = std::sqrt(target[0]*target[0] + target[1]*target[1]);
    double sign = (target[0] >= 0.0) ? 1.0 : -1.0;

    return magnitude * sign;
}

double Controller::PUpdate(const std::vector<double>& target) {
    error_ = signedError(target);
    double linear_fb = alpha_ * error_;

    return checkLimits(linear_fb);
}

double Controller::DSRUpdate(const std::vector<double>& target) {
    error_ = signedError(target);
    // TODO: finish DSR update 

    return 0.0;
}