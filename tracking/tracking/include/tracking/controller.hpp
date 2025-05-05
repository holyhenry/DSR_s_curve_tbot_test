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

    // System memories
    double error_;
    Eigen::MatrixXd observations_; // shape: N rows × 2 columns
    Eigen::Vector2d target_last_;  // 2D [x, y]
    Eigen::Vector2d state_last_;   // 2D [x, y]

    // ==============================Core functions==============================
    double angularUpdate(const Eigen::Vector2d& target) const;
    double PUpdate(const Eigen::Vector2d& target);
    double DSRUpdate(const Eigen::Vector2d& target);
    
    Eigen::Vector2d getTarget(bool memory_mode = true);
    void setObservations(const Eigen::MatrixXd& observations);
    std::vector<double> step(double linear_update, double angular_update);

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
    // Controller output - [v, w]
    Eigen::Vector2d input_;
    // Controller input limits
    double const vel_max_   = 0.2;
    double const vel_min_   = -0.2;
    double const omega_max_ = 1.0;
    double const omega_min_ = -1.0;

    // =============================Helper functions=============================
    double lowPass(double x, double x_last, double gain) const;
    double checkLimits(double u, double min, double max) const;
    double signedError(const Eigen::Vector2d& target) const;
    std::vector<double> toStdVector(const Eigen::Vector2d& v) const;
    
};