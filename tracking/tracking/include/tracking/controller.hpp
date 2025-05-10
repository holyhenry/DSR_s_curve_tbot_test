#pragma once

#include <ros/ros.h>
#include <numeric>
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

struct ControllerDebug
{
    // From DSRUpdate()
    Eigen::Vector2d target_last;
    double t_d_last;
    double s_d_last;
    double reinforce_term;
};

class Controller{
public:
    Controller(double alpha,
               double alpha_angle,
               double beta1,
               double beta2,
               double tau,
               double spacing);

    // ==============================Core functions==============================
    double angularUpdate(const Eigen::Vector2d& target) const;
    double PUpdate(const Eigen::Vector2d& target);
    double DSRUpdate(const Eigen::Vector2d& target, const Eigen::Vector2d& displacement);

    Eigen::Vector2d getTarget(bool memory_mode = true);
    void setObservations(const Eigen::MatrixXd& observations);
    std::vector<double> step(double linear_update, double angular_update);

    // ==========================Debug Helper functions==========================
    ControllerDebug getDebugData() const;

private:
    // Basic gains
    double alpha_;
    double alpha_angle_;
    // DSR gains
    double beta1_;
    double beta2_;
    // Controller parameters 
    double tau_;                       // DSR time delay (=controller period)
    double spacing_;                   // Inter-robot spacing
    // Controller command - [v, w]
    Eigen::Vector2d input_;
    // Controller input limits
    double const vel_max_   = 0.2;
    double const vel_min_   = -0.2;
    double const omega_max_ = 1.0;
    double const omega_min_ = -1.0;
    // Controller memories
    double error_;
    Eigen::MatrixXd observations_;     // N rows × 2 columns
    // DSR specific memories
    Eigen::Vector2d target_last_;
    double t_d_last_;                  // Target delayed term 
    double s_d_last_;                  // Self delayed term

    // =============================Helper functions=============================
    double lowPass(double x, double x_last, double gain) const;
    double checkLimits(double u, double min, double max) const;
    double signedError(const Eigen::Vector2d& target) const;
    std::vector<double> toStdVector(const Eigen::Vector2d& v) const;

};