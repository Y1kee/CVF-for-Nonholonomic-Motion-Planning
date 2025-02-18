#include "cvf_fix_wing/sacontrol_utils.h"

namespace sacontrol_utils {

Eigen::Vector2d CVF(const Eigen::Vector2d &p_2d, 
                    const Eigen::Vector2d &des_p_2d, 
                    const double &des_angle,
                    const double &rho)
{
    /* Spliting R^2 space into four regions depending on distance error */
    double r1 = 6  * rho;
    double r2 = 12 * rho;
    double r3 = 18 * rho;
    Eigen::Vector2d p_delta = des_p_2d - r2 * Eigen::Vector2d(std::cos(des_angle - M_PI/2), std::sin(des_angle - M_PI/2));
    Eigen::Vector2d q = p_2d - p_delta;
    double r_delta = q.norm();

    /* Flow field of source */
    Eigen::Vector2d F_out = q.normalized();
    /* Flow field of sink */
    Eigen::Vector2d F_in = -q.normalized();
    /* Flow field of vortex */
    Eigen::Vector2d F_v(-q(1), q(0));
    F_v.normalize();

    Eigen::Vector2d T;
    if (r_delta <= r1) {
        // flow field of the source 
        T = F_out;
    } else if (r_delta <= r2) {
        // mixture of source and vortex
        double lambda = mix((r_delta - r1) / (r2 - r1));
        T = lambda * F_out + (1 - lambda) * F_v;
    } else if (r_delta <= r3) {
        // mixture of vortex and sink
        double lambda = mix((r_delta - r2) / (r3 - r2));
        T = lambda * F_v + (1 - lambda) * F_in;
    } else {
        // flow field of the sink
        T = F_in;
    }
    return T.normalized();
}

double angular_velocity(const Eigen::Vector2d &p_2d, 
                        const double &theta, 
                        const Eigen::Vector2d &des_p_2d,
                        const double &des_angle,
                        const double &rho,
                        double &theta_e_tmp,
                        const double v_min,
                        const double k_v,
                        const double c_p,
                        const double c_theta,
                        const double kw_max)
{
    /* Citing the planning result for linear velocity */
    double vx = linear_velocity(p_2d, theta, des_p_2d, des_angle, rho, v_min, k_v, c_p, c_theta);

    /* Orientation error between Reference orientation specified by CVF */
    Eigen::Vector2d T = CVF(p_2d, des_p_2d, des_angle, rho);
    double theta_r = std::atan2(T(1), T(0));  // in [-pi,pi)
    double theta_e = std::fmod(theta-theta_r+M_PI, 2*M_PI);
    if (theta_e < 0 ) {
        theta_e += 2*M_PI;
    }
    theta_e -= M_PI;

    /* Time derivative of reference orientation under !vx! */
    double w_r = omega_r(p_2d, theta, vx, des_p_2d, des_angle, rho);  // 参考姿态变化率

    /* Saturated adaptive control law for angular velocity */
    double w0 = - kw(p_2d, theta, vx, des_p_2d, des_angle, rho, kw_max) * theta_e + w_r;

    // Eq.9
    double w = sat(w0,vx/rho,-vx/rho);

    //DBG
    theta_e_tmp = theta_e;
    return w;
}

double linear_velocity(const Eigen::Vector2d &p_2d,
                       const double &theta,
                       const Eigen::Vector2d &des_p_2d,
                       const double &des_angle,
                       const double &rho,
                       const double v_min,
                       const double k_v,
                       const double c_p,
                       const double c_theta)
{
    // Distance between current position and target position
    Eigen::Vector2d p_e = p_2d - des_p_2d;

    // Orientation error between Reference orientation specified by CVF
    Eigen::Vector2d T = CVF(p_2d, des_p_2d, des_angle, rho);
    double theta_r = std::atan2(T(1), T(0)); // in [-pi,pi)
    // Eq.7
    double theta_e = std::fmod(theta-theta_r+M_PI, 2*M_PI);
    if (theta_e < 0 ) {
        theta_e += 2*M_PI;
    }
    theta_e -= M_PI;

    // Planning result for linear velocity
    // Eq.8
    // double vx = v_min + k_v * std::tanh(p_e.norm() / c_p + std::abs(theta_e) / c_theta);
    double vx = v_min + k_v * (1 + 0.5*std::sin(-M_PI/2 + M_PI * std::tanh(p_e.norm() / c_p + std::abs(theta_e) / c_theta)))/2;

    return vx;
}

double dot_vx(const Eigen::Vector2d &p_2d,
              const double &theta,
              const Eigen::Vector2d &V_2d,
              const double &dot_yaw,
              const Eigen::Vector2d &des_p_2d,
              const double &des_angle,
              const double &rho,
              const double v_min,
              const double k_v,
              const double c_p,
              const double c_theta)
{
    // Distance between current position and target position
    Eigen::Vector2d p_e = p_2d - des_p_2d;

    // Orientation error between Reference orientation specified by CVF
    Eigen::Vector2d T = CVF(p_2d, des_p_2d, des_angle, rho);
    double theta_r = std::atan2(T(1), T(0)); // in [-pi,pi)
    double theta_e = std::fmod(theta-theta_r+M_PI, 2*M_PI);
    if (theta_e < 0 ) {
        theta_e += 2*M_PI;
    }
    theta_e -= M_PI;

    // Actual time derivative of theta_r
    double w_r_actual = omega_r(p_2d, theta, V_2d.norm(), des_p_2d, des_angle, rho);

    // d vx / dt
    // Equation 11
    // double dvx_dt = k_v * (1 - std::pow(std::tanh(p_e.norm() / c_d + std::abs(theta_e) / c_theta), 2)) *
                    // ((V_2d.dot(p_e)) / p_e.norm() / c_d + (dot_yaw - w_r_actual) / c_theta * std::copysign(1.0, theta_e));
    double dvx_dt = k_v * 0.5*M_PI*std::cos(-M_PI/2 + M_PI * std::tanh(p_e.norm() / c_p + std::abs(theta_e) / c_theta))
                    * (1 - std::pow(std::tanh(p_e.norm() / c_p + std::abs(theta_e) / c_theta), 2)) *
                    ((V_2d.dot(p_e)) / p_e.norm() / c_p + (dot_yaw - w_r_actual) / c_theta * std::copysign(1.0, theta_e));

    return dvx_dt;
}

double omega_r(const Eigen::Vector2d &p_2d,
               const double &theta,
               const double &vx,
               const Eigen::Vector2d &des_p_2d,
               const double &des_angle,
               const double &rho)
{
    /* Spliting R^2 space into four regions depending on distance error */
    double r1 = 6  * rho;
    double r2 = 12 * rho;
    double r3 = 18 * rho;

    /* Distance to The singular point of the CVF */
    Eigen::Vector2d p_delta = des_p_2d - r2 * Eigen::Vector2d(std::cos(des_angle - M_PI/2), std::sin(des_angle - M_PI/2));
    Eigen::Vector2d q = p_2d - p_delta;
    double r_delta = q.norm();

    /* Decomposion the planned linear velocity along the direction of q=p_2d-p_delta */
    Eigen::Vector2d dot_p_2d = vx * Eigen::Vector2d(std::cos(theta), std::sin(theta));
    double vx_paral = q.normalized().dot(dot_p_2d);
    Eigen::Matrix2d m;
    m << 0, -1, 1, 0;
    double vx_ortho = (m * q.normalized()).dot(dot_p_2d);

    /* The time derivative of reference orientation */
    // Eq.10
    double w_r = 1 / r_delta * vx_ortho + ptpr(r_delta, rho) * vx_paral;

    return w_r;
}

// partial derivative of theta_r w.r.t. r_delta
double ptpr(double r_delta, double rho) {
    // Eq.16 
    double r1 = 6 * rho;
    double r2 = 12 * rho;
    double r3 = 18 * rho;
    double g = 0;

    if (r_delta <= r1) {
        g = 0;
    } else if (r_delta <= r2) {
        double s2 = (r_delta - r1) / (r2 - r1);
        double lambda = mix(s2);
        g = (6 * s2 - 6 * s2 * s2) / (r2 - r1) / (2 * lambda * lambda - 2 * lambda + 1);
    } else if (r_delta <= r3) {
        double s3 = (r_delta - r2) / (r3 - r2);
        double lambda = mix(s3);
        g = (6 * s3 - 6 * s3 * s3) / (r3 - r2) / (2 * lambda * lambda - 2 * lambda + 1);
    } else {
        g = 0;
    }

    return g;
}

/* Compute the adaptive gain for angular velocity control. */
double kw(const Eigen::Vector2d& p_2d, 
          const double &theta, 
          const double &vx, 
          const Eigen::Vector2d &des_p_2d, 
          const double &des_angle, 
          const double &rho,
          const double kw_max) 
{
    // Distance between the current and singular point
    double r2 = 12 * rho;
    Eigen::Vector2d p_delta = des_p_2d - r2 * Eigen::Vector2d(std::cos(des_angle - M_PI / 2), std::sin(des_angle - M_PI / 2));
    Eigen::Vector2d q = p_2d - p_delta;
    double r_delta = q.norm();

    // Maximum curvature (inverse of the minimum turning radius)
    double K = 1 / rho;

    // Compute the curvature-constrained vector field (CVF) direction
    Eigen::Vector2d T = CVF(p_2d, des_p_2d, des_angle, rho);
    double theta_r = std::atan2(T(1), T(0));
    double theta_e = std::fmod(theta-theta_r+M_PI, 2*M_PI);
    if (theta_e < 0 ) {
        theta_e += 2*M_PI;
    }
    theta_e -= M_PI;

    // Angle theta_inc is the direction where theta_r increases the fastest
    double phi_delta = std::atan2(q(1), q(0));
    double theta_inc = phi_delta + std::atan2(1 / r_delta, ptpr(r_delta, rho));

    // k0: Intermediate gain calculation based on curvature, velocity, and angular errors
    // Eq.13
    double k0 = (K - cal_k(r_delta, rho) * std::abs(std::cos(theta - theta_inc))) / std::abs(theta_e) * vx;
 
    // 角速度的自适应增益，受kw_max约束
    return std::min(kw_max, k0);
}


/* the nonlinear function used in adaptive gain Equation 14 */
double cal_k(double r_delta, double rho) 
{
    if (r_delta >= rho) {
        return 1 / r_delta + ptpr(r_delta, rho);
    } else {
        return r_delta / (rho * rho);
    }
}



} // namespace sacontrol_utils
