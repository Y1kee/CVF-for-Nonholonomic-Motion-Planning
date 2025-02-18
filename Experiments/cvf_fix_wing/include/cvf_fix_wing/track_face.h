#ifndef __TRACK_FACE_H
#define __TRACK_FACE_H
#include "cvf_fix_wing/track_common.h"
#include "cvf_fix_wing/visualizer.hpp"
#include "friendly_code/colorful_cout_define.h"
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <algorithm>

#include <cvf_fix_wing/dbg_msg.h>

class TrackFaceMonitor
{
public:
  TrackFaceMonitor(ros::NodeHandle &nh) : nh_(nh)
  {
  }
  ros::NodeHandle nh_;
};


struct curState
{
  Eigen::Vector3d cur_pos;      // current position in world frame, \in R^3
  Eigen::Quaterniond cur_ori;   // current attitude in quaternion, \in R^4
  Eigen::Vector3d cur_vel;      // current velocity in world frame, \in R^3
  Eigen::Vector3d cur_vel_body; // current velocity in body frame, \in R^3
  Eigen::Vector3d cur_Omega_body;
  Eigen::Vector3d cur_Omega;
};


class TrackFace
{
public:
  TrackFace(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  bool SAControl(const curState &s, const Eigen::Vector2d &des_pos_2d,
                 const double &des_angle, const double &height, const double &rho,
                 Eigen::Quaterniond &q_cmd, double &thrust_cmd);

private:
  ros::NodeHandle nh, pnh;
  Visualizer visualizer;
  TrackFaceMonitor monitor; // for monitoring and recording data

  ros::Subscriber state_sub;
  ros::Subscriber odom_sub;

  ros::Publisher attitude_cmd_pub;
  ros::Publisher local_pos_cmd_pub;

  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;

  ros::Time last_request;
  ros::Timer timer_;
  double control_dt_;

  coopfly::TRACK_STATE track_state;
  mavros_msgs::State current_state;
  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::CommandBool arm_cmd;

  // vechile params
  int uav_id_;
#if (FLIGHT_TYPE == 1)
  const double mass = 1.505; //uav mass，QiaoYk
#elif (FLIGHT_TYPE == 2)
  const double mass = 1.0; //uav mass
#endif

  // control params
  Eigen::Vector3d circle_pos_cmd_{0, 0, 0};
  Eigen::Vector2d des_p_;           
  double des_angle_, flight_height_, rho_; 

  // const double rho = 10.0; // 最小转弯半径

  // global variables
  curState s; // current states
  int armed_snap{0}, armed_snap_max;
  int uninterrupted_call_times{0};
  int max_uninterrupted_call_times;
  bool start_tracking_{false}, has_odom_{false};
  bool have_to_circle_at_first_wp_{false}, first_circle_{false};

  // visualization
  tf::TransformBroadcaster br;
  tf::Transform transform;

  // functions
  void updateCallback(const ros::TimerEvent &e);
  void stateCallback(const mavros_msgs::State::ConstPtr &msg);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  bool trackingTrajectory();

  // Fixed-wing Parameters
  double wing_area;
  double s_total;
  double alpha0;
  double cla;
  double cla_stall;
  double alpha_stall;
  double cda;
  double cda_stall;
  double cl;

  // Control law parameters
  double v_max;
  double v_min;
  double k_v;
  double c_p;
  double c_theta;

  // Aero parameters
  double rho_air;

  // Maximum allowable gain for angular velocity
  double kw_max;

#if (DBG == 1)
  ros::Publisher dbg_pub;
#endif

  inline double accToThrSignal(const double &acc)
  {
    const double f = acc * mass;
#if (FLIGHT_TYPE == 1)
    return thrToCmdGazebo(f);
#elif (FLIGHT_TYPE == 2)
    return thrToCmdHIL(f);
#endif
  }
  inline double thrToCmdGazebo(const double &thr)
  {
    constexpr double max_n = 3500;
    double n = thr > 0 ? 515.616339782699 * sqrt(thr) : 0; // 电机转速
    double cmd = n > max_n ? 1 : n / max_n;
    return cmd;
  }

  inline double thrToCmdHIL(const double &thr)
  {
    // constexpr double max_thr = 3.837;
    double cmd = 0.0577 * thr + 0.02196;
    cmd = cmd > 1 ? 1 : cmd;
    cmd = cmd < 0 ? 0 : cmd;
    return cmd;
  }
  inline bool checkFcuState()
  {
    /** Check and set offboard mode **/
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      else
      {
        ROS_ERROR("Failed to set offboard mode");
        return false;
      }
      last_request = ros::Time::now();
    }
    /** Check and set armed **/
    else
    {
      if (!current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        else
        {
          ROS_ERROR("Failed to arm vehicle");
          return false;
        }
        last_request = ros::Time::now();
      }
    }
    return true;
  }
  inline void changeTrackStateTo(coopfly::TRACK_STATE new_state)
  {
    std::cout << GREEN << "[tracker] Change state from "
              << coopfly::track_state_str[int(track_state)] << " to "
              << coopfly::track_state_str[int(new_state)] << RESET << std::endl;
    track_state = new_state;
  }

  inline void publishPosCmd(const Eigen::Vector3d &pos)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = pos(0);
    pose.pose.position.y = pos(1);
    pose.pose.position.z = pos(2);
    local_pos_cmd_pub.publish(pose);
  }

  inline void publishOmgCmd(const Eigen::Quaterniond &q_r,
                            const double &thr_sig)
  {
    mavros_msgs::AttitudeTarget att_cmd;
    att_cmd.header.stamp = ros::Time::now();

    att_cmd.orientation.w = q_r.w();
    att_cmd.orientation.x = q_r.x();
    att_cmd.orientation.y = q_r.y();
    att_cmd.orientation.z = q_r.z();

    att_cmd.thrust = thr_sig;

    att_cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;

    attitude_cmd_pub.publish(att_cmd);
    // std::cout << "att cmd, omg:" << att_cmd.body_rate.x << " " << att_cmd.body_rate.y  << " " <<
    // att_cmd.body_rate.z  << ", thr:" << att_cmd.thrust << std::endl;
  }

  inline bool circlingAtFirstWp()
  {
    first_circle_ = first_circle_ || (s.cur_pos - circle_pos_cmd_).norm() < 10;
    return first_circle_;
  }
  inline void calAeroForceHil(const double &speed,
                              const double alpha,
                              double &lift,
                              double &drag)
  {
    constexpr double aircraftS = 1.0;
    constexpr double aircraftCL0 = 0.14684;
    constexpr double aircraftCLa = 1.4456;
    constexpr double aircraftCd0 =  0.035362 + 0.0004;
    constexpr double aircraftCda =  0.11912;
    constexpr double rho = 1.225;
    lift = 0.5 * rho * aircraftS * speed * speed * (aircraftCLa * alpha + aircraftCL0);
    drag = 0.5 * rho * aircraftS * speed * speed * (aircraftCda * alpha + aircraftCd0 + 0.002 * (speed-14));
  }
  // modified from gazebo lifgdrag_plugin.cpp for coordinated flight model
  inline void calAeroForceGazebo(const double &speed,
                                const double attack_angle,
                                double &lift,
                                double &drag)
  {
    double alpha = alpha0 + attack_angle;

    if (alpha > alpha_stall)
    {
        cl = (cla * alpha_stall +
              cla_stall * (alpha - alpha_stall));
        // make sure cl is still great than 0
        cl = std::max(0.0, cl);
    }
    else if (alpha < -alpha_stall)
    {
        cl = (-cla * alpha_stall +
              cla_stall * (alpha + alpha_stall));
        // make sure cl is still less than 0
        cl = std::min(0.0, cl);
    }
    else
        cl = cla * alpha;
    lift = 0.5 * rho_air * speed * speed * s_total * cl;

    // compute cd at cp, check for stall, correct for sweep
    double cd;
    if (alpha > alpha_stall)
    {
        cd = (cda * alpha_stall +
              cda_stall * (alpha - alpha_stall));
    }
    else if (alpha < -alpha_stall)
    {
        cd = (-cda * alpha_stall +
              cda_stall * (alpha + alpha_stall));
    }
    else
        cd = (cda * alpha);

    // make sure drag is positive
    cd = fabs(cd);

    // drag at cp
    drag = cd * 0.5 * rho_air * speed * speed * s_total;
  }
};

#endif