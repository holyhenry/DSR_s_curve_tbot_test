#pragma once

#include <ros/ros.h>
#include <numeric>
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

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

    // System memories
    double error_;
    Eigen::MatrixXd observations_;     // shape: N rows × 2 columns
    std::vector<double> target_last_;  // 2D [x, y]
    std::vector<double> state_last_;   // 2D [x, y]

    // ==============================Core functions==============================
    std::vector<double> getTarget(bool memory_mode = true);
    double angularControl(const std::vector<double>& state,
                          const std::vector<double>& target,
                          double velocity) const;
    double PUpdate(const std::vector<double>& target);
    double DSRUpdate(const std::vector<double>& target);

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
    // DSR specific memories
    double t_d_last_;                  // Self-delayed
    double e_l_last_;                  // Predecessor delayed
    // Controller output
    std::vector<double> input_;        // 2D [v, w]
    // Controller output limits
    double const max_ = 0.2;
    double const min_ = -0.2;

    // =============================Helper functions=============================
    double lowPass(double x, double x_last, double gain) const;
    double checkLimits(double u) const;
    double signedError(const std::vector<double>& target) const;
    
};