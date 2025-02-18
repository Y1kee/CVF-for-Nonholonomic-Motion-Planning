#ifndef __SACONTROL_UTILS_H
#define __SACONTROL_UTILS_H

#include <Eigen/Dense>
#include <algorithm>

namespace sacontrol_utils {

// Impose upper bound ub and lower bound lb on the scalar x
inline double sat(const double &x, const double &ub, const double &lb)
{
    return std::max(std::min(x, ub), lb);
}

// Blendding function smooth on decrease from 1 to 0 on [0,1]
inline double mix(double x) {
    // Eq.15
    return 2 * std::pow(x, 3) - 3 * std::pow(x, 2) + 1;
}

/* CVF */
Eigen::Vector2d CVF(const Eigen::Vector2d &p_2d, 
                    const Eigen::Vector2d &des_p_2d, 
                    const double &des_angle,
                    const double &rho);


/* angular_velocity */
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
                        const double kw_max);


/* linear_velocity */
double linear_velocity(const Eigen::Vector2d &p_2d,
                       const double &theta,
                       const Eigen::Vector2d &des_p_2d,
                       const double &des_angle,
                       const double &rho,
                       const double v_min,
                       const double k_v,
                       const double c_p,
                       const double c_theta);

/* dot_vx */
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
              const double c_theta);

/* omega_r */
double omega_r(const Eigen::Vector2d &p_2d,
               const double &theta,
               const double &vx,
               const Eigen::Vector2d &des_p_2d,
               const double &des_angle,
               const double &rho);

/* ptpr */
double ptpr(double r_delta, double rho);

/* Compute the adaptive gain for angular velocity control. */
double kw(const Eigen::Vector2d& p_2d, 
          const double &theta, 
          const double &vx, 
          const Eigen::Vector2d &des_p_2d, 
          const double &des_angle, 
          const double &rho,
          const double kw_max);

/* the nonlinear function used in adaptive gain Equation 14 */
double cal_k(double r_delta, double rho);

} // namespace sacontrol_utils

#endif // __SACONTROL_UTILS_H