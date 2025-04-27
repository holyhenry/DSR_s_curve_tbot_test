#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>

class Controller{
public:
    Controller(double alpha,
               double alpha_angle,
               double beta1,
               double beta2,
               double tau,
               double spacing);

    // Main control interface
    std::pair<double, double> step(const std::vector<double>& current_state,
                                   const std::vector<double>& target,
                                   double velocity);

private:
    // Basic gains
    double alpha_;
    double alpha_angle_;
    // Additional gains for DSR
    double beta1_;
    double beta2_;
    // System parameters 
    double tau_;                       // DSR delay (=controller period)
    double spacing_;                   // Inter-robot spacing
    // System memories
    double error_;
    std::vector<double> target_last_;  // 2D
    std::vector<double> state_last_;   // 2D
    // DSR specific memories
    double t_d_last_;                  // Self-delayed
    double e_l_last_;                  // Predecessor delayed
    // Controller output
    std::vector<double> input_;        // 2D [v, w]
    // Controller output limits
    double max_ = 0.2;
    double min_ = -0.2;

    // Utility functions
    double lowPass(double x, double x_last, double gain) const;

    double checkLimits(double u) const;

    double signedError(const std::vector<double>& target) const;

    // Core logic
    double angularControl(const std::vector<double>& state,
                          const std::vector<double>& target,
                          double velocity) const;

    double PUpdate(const std::vector<double>& target);

    double DSRUpdate(const std::vector<double>& target);

    
};