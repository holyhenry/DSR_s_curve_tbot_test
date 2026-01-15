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

void Controller::interpTraj(const Eigen::Vector2d& predecessor_state, int pts)
{
    pts = std::max(2, pts);

    // Self-state is always zeros in current local frame
    const Eigen::Vector2d start = Eigen::Vector2d::Zero();  
    const Eigen::Vector2d end   = predecessor_state;
    const Eigen::Vector2d start2 = 2.0 * start - end; // mirrored start pt for virtual tail traj

    observations_.resize(pts, 2);
    for (int i = 0; i < pts; ++i){
        const double s = static_cast<double>(i) / static_cast<double>(pts - 1);
        const Eigen::Vector2d p = (1.0 - s) * start2 + s * end;
        observations_.row(i) = p.transpose();
    }
}

double Controller::angularUpdate(const Eigen::Vector2d& target) const
{
    double theta = std::atan2(target[1], target[0]);

    // Stop rotating if the angular error is too large (±60 deg)
    if (std::abs(theta) > M_PI / 3.0){
        return 0.0;
    }

    double omega_fb = alpha_angle_ * theta;
    return checkLimits(omega_fb, omega_min_, omega_max_);
}

double Controller::trajAngularUpdate(double v_current, int degree,
                                     double weight_factor, int stabilizing_tail)
{
    if (observations_.rows() < (degree + 1)) {
        ROS_WARN("[trajAngularUpdate] Not enough observations for poly fit.");
        return 0.0;
    }

    const TrajDerivatives2D d = fitPolyTrajectory(degree, weight_factor, stabilizing_tail);

    const double dx_dt = d.dx_dt;
    const double dy_dt = d.dy_dt;
    const double ddx_dt = d.ddx_dt;
    const double ddy_dt = d.ddy_dt;

    // Compute feedforward
    const double v_tau2 = dx_dt * dx_dt + dy_dt * dy_dt;
    if (v_tau2 < 1e-10) { return 0.0; }
    const double v_tau = std::sqrt(v_tau2);
    const double curvature_numerator = dx_dt * ddy_dt - dy_dt * ddx_dt;
    const double omega_ff = v_current * curvature_numerator / (v_tau2 * v_tau);

    // Compute feedback
    const double traj_yaw = std::atan2(dy_dt, dx_dt);
    const double yaw_err = wrapToPi(traj_yaw);
    const double omega_fb = angularGainMap(v_current, 0.01) * yaw_err;

    return checkLimits(omega_ff + omega_fb, omega_min_, omega_max_);
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
    const double lp_gain = 0.2;

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
    double s_d = lowPass(delta_state / tau_, s_d_last_, 1.0);
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

double Controller::wrapToPi(double a)
{
    if (a > M_PI) a -= 2.0 * M_PI;
    if (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double Controller::angularGainMap(double velocity, double threshold) const
{
    return (std::abs(velocity) <= threshold) ? 0.0 : alpha_angle_;
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

Eigen::MatrixXd Controller::getFitStates(int* query_idx, int stabilizing_tail)
{
    const int N = static_cast<int>(observations_.rows());
    if (N <= 0){
        ROS_WARN("[getFitStates] observations_ is empty");
        if (query_idx) *query_idx = 0;
        return Eigen::MatrixXd(0, 2);
    }

    // 1) Vectorized distance-to-origin argmin: d2 = x^2 + y^2
    const Eigen::ArrayXd d2 = observations_.col(0).array().square() +
                              observations_.col(1).array().square();

    Eigen::Index idx = 0;
    double min_d2 = d2.minCoeff(&idx);
    const int argmin = static_cast<int>(idx);

    // 2) Fit window start (stabilizing_tail points behind the closest point)
    const int start_idx = std::max(argmin - stabilizing_tail, 0); 
    const int n_fit = N - start_idx;
    Eigen::MatrixXd fit_states = observations_.block(start_idx, 0, n_fit, 2);

    // Query index inside fit window (clamped, technically = stabilizing_tail)
    if (query_idx) *query_idx = std::max(std::min(stabilizing_tail, n_fit - 1), 0);
    ROS_INFO_STREAM("query_idx !!!: " << *query_idx 
                                      << " fit_states: " << fit_states.rows());

    // 3) MEMORY TRIMMING ([deprecated] move trimming part to tracking_node.cpp)
    // if (start_idx > 50){
    //     const int keep_from = start_idx - 50;
    //     observations_ = observations_.block(keep_from, 0, N - keep_from, 2);
    // }

    return fit_states;
}

TrajDerivatives2D Controller::fitPolyTrajectory(int degree, double weight_factor,
                                                int stabilizing_tail)
{
    TrajDerivatives2D out;

    int query_idx;
    const Eigen::MatrixXd fit_states = getFitStates(&query_idx, stabilizing_tail);

    // Set up the weighted least squares problem & Vandermonde matrix
    const int n = static_cast<int>(fit_states.rows());
    if (n < degree + 1) {
        ROS_WARN_STREAM("[fitPolyTrajectory] Not enough points for poly fit: n="
                        << n << ", degree=" << degree);
        return out;
    }

    Eigen::VectorXd t(n);  // Time grid t in [0, 1]
    const double inv = 1.0 / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) t(i) = static_cast<double>(i) * inv;
    const double t_query = t(query_idx);

    Eigen::MatrixXd V(n, degree + 1);  // Build Vandermonde V (increasing powers)
    V.col(0).setOnes();
    for (int k = 1; k <= degree; ++k) {
        V.col(k) = V.col(k - 1).cwiseProduct(t);
    }

    // Create weights: higher weight for the closest state
    Eigen::VectorXd w = Eigen::VectorXd::Ones(n);
    w(query_idx) = weight_factor;  // w(0) = weight_factor;

    // Solve (W @ V) @ coeffs = W @ observed_values  
    // Apply weights row-wise to avoid forming a diagonal matrix.
    Eigen::MatrixXd WV = V;
    Eigen::VectorXd Wx(n), Wy(n);
    for (int i = 0; i < n; ++i) {
        const double wi = w(i);
        WV.row(i) *= wi;
        Wx(i) = wi * fit_states(i, 0);  // x
        Wy(i) = wi * fit_states(i, 1);  // y
    }
    const Eigen::VectorXd coeffs_x = WV.colPivHouseholderQr().solve(Wx);
    const Eigen::VectorXd coeffs_y = WV.colPivHouseholderQr().solve(Wy);

    // Evaluate derivatives at tq without building extra polynomial objects.
    // For coefficients c0..cd:
    // dx/dt  = sum_{k=1..d} k*c_k*t^(k-1)
    // d2x/dt2= sum_{k=2..d} k*(k-1)*c_k*t^(k-2)
    auto eval_d1 = [](const Eigen::VectorXd& c, double x) {
        double s = 0.0;
        double xpow = 1.0; // x^(k-1), starts at k=1 -> x^0
        for (int k = 1; k < c.size(); ++k) {
            s += static_cast<double>(k) * c(k) * xpow;
            xpow *= x;
        }
        return s;
    };
    auto eval_d2 = [](const Eigen::VectorXd& c, double x) {
        double s = 0.0;
        double xpow = 1.0; // x^(k-2), starts at k=2 -> x^0
        for (int k = 2; k < c.size(); ++k) {
            s += static_cast<double>(k) * static_cast<double>(k - 1) * c(k) * xpow;
            xpow *= x;
        }
        return s;
    };

    out.dx_dt  = eval_d1(coeffs_x, t_query);
    out.ddx_dt = eval_d2(coeffs_x, t_query);
    out.dy_dt  = eval_d1(coeffs_y, t_query);
    out.ddy_dt = eval_d2(coeffs_y, t_query);

    return out;
}

// ==========================Debug Helper functions==========================

ControllerDebug Controller::getDebugData() const
{
    return { error_last_, target_last_, t_d_last_, s_d_last_, e_d_last_, r_t_last_ };
}
