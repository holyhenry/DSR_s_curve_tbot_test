#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>

class Controller{
public:
    Controller(double dt,
               double alpha,
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
    // Parameters
    double dt_;
    double alpha_;
    double alpha_angle_;
    double beta1_;
    double beta2_;
    double tau_;
    double spacing_;

    // System state memory
    std::vector<double> target_last_;  // 2D
    std::vector<double> state_last_;   // 2D

    // Output
    std::vector<double> input_;  // [v, w]

    // System limits
    const double max_ = 0.2;
    const double min_ = -0.2;

    // Core logic
    double signedError(const std::vector<double>& state,
                       const std::vector<double>& target,
                       double heading_rad) const;

    double lowPass(double x, double x_last, double gain) const;

    double angularControl(const std::vector<double>& state,
                          const std::vector<double>& target,
                          double velocity) const;

    double PUpdate(const std::vector<double>& state,
                   const std::vector<double>& target,
                   double heading) const;

    double DSRUpdate(const std::vector<double>& state,
                     const std::vector<double>& target,
                     double heading);

    double checkLimits(double u) const;
};