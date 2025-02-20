
#include "cvf_fix_wing/track_face.h"
#include "cvf_fix_wing/sacontrol_utils.h"

#include <iostream>

using namespace coopfly;
TrackFace::TrackFace(ros::NodeHandle &nh_in, ros::NodeHandle &pnh_in)
    : nh(nh_in), pnh(pnh_in), visualizer(nh_in), monitor(nh)
{
  pnh.param("control_dt", control_dt_, 0.01);
  pnh.param("have_to_circle_at_first_wp", have_to_circle_at_first_wp_, false);
  pnh.param("uav_id", uav_id_, 0);

  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1,
                                               &TrackFace::stateCallback, this);

  odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "mavros/local_position/odom", 1, &TrackFace::odomCallback, this,
      ros::TransportHints().tcpNoDelay());

  local_pos_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 1);
  attitude_cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
      "mavros/setpoint_raw/attitude", 1);

  arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  timer_ = nh.createTimer(ros::Duration(control_dt_),
                          &TrackFace::updateCallback, this);

  last_request = ros::Time::now();

  offb_set_mode.request.custom_mode = "OFFBOARD";
  arm_cmd.request.value = true;

  // readin desire state
  pnh.param("des_px", des_p_(0), 0.0);
  pnh.param("des_py", des_p_(1), 0.0);
  pnh.param("des_angle", des_angle_, 0.0);
  pnh.param("rho", rho_, 10.0);
  pnh.param("flight_height", flight_height_, 20.0);
  pnh.param("defalut_circle_point_x", circle_pos_cmd_(0), 0.0);
  pnh.param("defalut_circle_point_y", circle_pos_cmd_(1), 0.0);
  circle_pos_cmd_(2) = flight_height_;

  double armed_snap_time;
  pnh.param("armed_snap_time", armed_snap_time, 1.0);
  armed_snap_max = armed_snap_time / control_dt_;

  track_state = INIT;

  double fixedwing_color[3];
  pnh.param("fixedwing_color_r", fixedwing_color[0], 0.0);
  pnh.param("fixedwing_color_g", fixedwing_color[1], 1.0);
  pnh.param("fixedwing_color_b", fixedwing_color[2], 0.0);

  visualizer.setFixedWingMeshSource(
      "package://cvf_fix_wing/meshes/fixedwing.stl");
  visualizer.setFixedWingColor(fixedwing_color[0],
                               fixedwing_color[1],
                               fixedwing_color[2]);

  // Fixed wing parameters
  pnh.param("wing_area", wing_area, 0.6);
  s_total = wing_area * 4;
  pnh.param("alpha0", alpha0, -0.05984281113);
  pnh.param("cla", cla, 4.752798721);
  pnh.param("cla_stall", cla_stall, -3.85);
  pnh.param("alpha_stall", alpha_stall, 0.3391428111);
  pnh.param("cda", cda, 0.6417112299);
  pnh.param("cda_stall", cda_stall, -0.9233984055);       
  pnh.param("cl", cl, 0.0);

  // Control law parameters
  pnh.param("v_max", v_max, 18.0);
  pnh.param("v_min", v_min, 13.0);
  k_v = v_max - v_min;
  pnh.param("c_p", c_p, 120.0);
  pnh.param("c_theta", c_theta, M_PI);

  // Aero parameters
  pnh.param("rho_air", rho_air, 1.2);    

  // Maximum allowable gain for angular velocity   
  pnh.param("kw_max", kw_max, 0.2);

#if (DBG == 1)
  dbg_pub = nh.advertise<cvf_fix_wing::dbg_msg>("/dbg_msg", 1);
#endif
}

void TrackFace::updateCallback(const ros::TimerEvent &e)
{
  if (!has_odom_)
  {
    ROS_WARN("Waiting for first odom");
    return;
  }

#if (FLIGHT_TYPE == 1)
  // sitl
  if (!checkFcuState())
  {
    publishPosCmd(circle_pos_cmd_);
    std::cout << "publishPosCmd" << std::endl;
    return;
  }

  // wait seconds after first armed
  if (current_state.armed && armed_snap < armed_snap_max)
  {
    armed_snap++;
    return;
  }

#elif (FLIGHT_TYPE == 2)
  // hil
  /** If mavros is not connected, no cmd is sent**/
  if (!current_state.connected)
  {
    ROS_WARN("mavros is not nonnected!");
    return;
  }
  /* 非offboard模式下，低于15m的时候，不切入offboard模式 */
  if (current_state.mode != "OFFBOARD" && s.cur_pos(2) < 15.0)
    return;

  if (!checkFcuState())
  {
    publishPosCmd(circle_pos_cmd_);
    return;
  }

#endif

  switch (track_state)
  {
  case INIT:
  {
    // do nothing, directly into circle_wait state
    changeTrackStateTo(CIRCLE_WAITING);
  }
  case CIRCLE_WAITING:
  {
    if (have_to_circle_at_first_wp_ && !circlingAtFirstWp())
    {
      publishPosCmd(circle_pos_cmd_);
      break;
    }
    else
    {
      changeTrackStateTo(TRACKING);
      first_circle_ = false;
    }
    break;
  }
  case TRACKING:
  {
    start_tracking_ = true;
    bool track_finished = trackingTrajectory();
    if (track_finished)
    {
      std::cout << "TRACKING finished" << std::endl;
      changeTrackStateTo(CIRCLE_WAITING);
    }
    break;
  }
  default:
    break;
  }
}

bool TrackFace::trackingTrajectory()
{
  Eigen::Quaterniond q_cmd;
  double thrust_cmd, thr_cmd;

  

  if (SAControl(s, des_p_, des_angle_, flight_height_, rho_, q_cmd, thrust_cmd))
  {

#if (FLIGHT_TYPE == 1)
    thr_cmd = thrToCmdGazebo(thrust_cmd);
#elif (FLIGHT_TYPE == 2)
    thr_cmd = thrToCmdHIL(thrust_cmd);
#endif

    publishOmgCmd(q_cmd, thr_cmd);
    return false; // not finished yet, kept tracking trajectory
  }
  else
  {
    return true; // finished already
  }
}

/* SAControl for QiaoYk's algorithm
 * Input: s, current state, please refer to curState struct
          des_pos_2d, desired position \in R^2 (m)
          des_angle, desired angle \in R (rad)
          des_height, desired height \in R (m)
 * Output:omg_cmd, omega command (rad/s)
          thrust_cmd, thrust command (N)
 */

bool TrackFace::SAControl(const curState &s, const Eigen::Vector2d &des_pos_2d,
                          const double &des_angle, const double &des_height, const double &rho,
                          Eigen::Quaterniond &q_cmd, double &thrust_cmd)
{
  /*******************/
  /* State variables */
  /*******************/
  Eigen::Vector3d p =s.cur_pos;
  double x = p(0);
  double y = p(1);
  double h = p(2);
  Eigen::Vector2d p_2d(x, y);

  Eigen::Quaterniond q = s.cur_ori;
  // Eigen::Vector3d eulerAngle = q.toRotationMatrix().eulerAngles(2, 1, 0);  // RPY
  // Eigen::Vector3d eulerAngle = q.toRotationMatrix().eulerAngles(2, 1, 0);  // RPY
  // double roll = eulerAngle(0);
  // double pitch = eulerAngle(1);
  // double yaw = eulerAngle(2);

  // double yaw = eulerAngle(0);
  // double pitch = eulerAngle(1);
  // double roll = eulerAngle(2);

  Eigen::Matrix3d rotm = q.toRotationMatrix();
  double roll = std::atan2(rotm(2,1),rotm(2,2));
  double pitch = std::atan2( -rotm(2,0) , std::sqrt(rotm(2,1)*rotm(2,1) + rotm(2,2)*rotm(2,2)) );
  double yaw = std::atan2(rotm(1,0),rotm(0,0));

  Eigen::Vector3d dot_p = s.cur_vel;
  Eigen::Vector2d dot_p_2d(dot_p(0), dot_p(1));

  // Eq.1
  double theta = std::atan2(dot_p(1), dot_p(0));

  // Eigen::Vector3d Omega_b = s.cur_vel_body;
  Eigen::Vector3d Omega_b = s.cur_Omega_body;

  // Eq.2
  double dot_yaw = (Omega_b(2) * std::cos(roll) - Omega_b(1) * std::sin(roll)) / std::cos(pitch);
  // std::cout << "dot_yaw: " << dot_yaw << std::endl; 
  double V = dot_p.norm();


  /******************************/
  /* Flight height control loop */
  /******************************/
  double K_pitch = 0.7;
  double pitch_max = M_PI / 6;
  double e_h = h - des_height;
  // Eq.3 
  double pitch_d = sacontrol_utils::sat(K_pitch*e_h/V, pitch_max, -pitch_max);  // setpoint for pitch angle

  /********************/
  /* Yaw control loop */
  /********************/
  Eigen::Vector2d T = sacontrol_utils::CVF(p_2d, des_pos_2d, des_angle, rho);


  // Eq.4
  double yaw_d = std::atan2(T(1), T(0));


  /*********************/
  /* Roll control loop */
  /*********************/
  double g = 9.8; // Gravitational acceleration TODO:Put in Parameters
  double theta_e_tmp = 0.0;
  double omega_planned = sacontrol_utils::angular_velocity(p_2d, theta, des_pos_2d, des_angle, rho, theta_e_tmp, v_min, k_v, c_p, c_theta, kw_max);  // planning result for angular velocity
  // Eq.5
  // double roll_d = - std::atan2(omega_planned * V, g);  // setpoint for roll angle
  double roll_d = - std::atan2(omega_planned * V , g * std::cos(pitch));  // setpoint for roll angle




  /***************************************/
  /* Setpoint for attitude in quaternion */
  /***************************************/
  Eigen::Matrix3d rot_mat;
  // rot_mat = Eigen::AngleAxisd(roll_d, Eigen::Vector3d::UnitX())
  //         * Eigen::AngleAxisd(pitch_d, Eigen::Vector3d::UnitY())
  //         * Eigen::AngleAxisd(yaw_d, Eigen::Vector3d::UnitZ());
  rot_mat = Eigen::AngleAxisd(yaw_d, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(pitch_d, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(roll_d, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q_d(rot_mat);


  /***********************/
  /* Thrust control loop */
  /***********************/
  double K_T = 14;  // Gain for thrust

  double v_planned = sacontrol_utils::linear_velocity(p_2d, theta, des_pos_2d, des_angle, rho, v_min, k_v, c_p, c_theta);
  double dvx_dt = sacontrol_utils::dot_vx(p_2d,theta,dot_p_2d,dot_yaw,des_pos_2d,des_angle,rho, v_min, k_v, c_p, c_theta);
  double e_v = dot_p_2d.norm() - v_planned;

  /**************/
  /* Drag Model */
  /**************/
  double drag = 0.0;  
  double lift = 0.0;
  double speed = s.cur_vel.norm();
  double attack_angle = -std::atan2(s.cur_vel_body(2), s.cur_vel_body(0));

#if (FLIGHT_TYPE == 1)
  calAeroForceGazebo(speed, attack_angle, lift, drag);
#elif (FLIGHT_TYPE == 2)
  calAeroForceHil(speed, attack_angle, lift, drag);
#endif

  /**********/
  /* Output */
  /**********/
  q_cmd = q_d;
  // Eq.6
  thrust_cmd = std::max(0.0, (-K_T*e_v + dvx_dt)*mass + drag);

#if (DBG == 1)  // Debug
  cvf_fix_wing::dbg_msg dbg_msg;
  dbg_msg.header.stamp = ros::Time::now();

  dbg_msg.cur_pos.x = s.cur_pos(0);
  dbg_msg.cur_pos.y = s.cur_pos(1);
  dbg_msg.cur_pos.z = s.cur_pos(2);

  dbg_msg.cur_ori.w = s.cur_ori.w();
  dbg_msg.cur_ori.x = s.cur_ori.x();
  dbg_msg.cur_ori.y = s.cur_ori.y();
  dbg_msg.cur_ori.z = s.cur_ori.z();

  dbg_msg.cur_vel.x = s.cur_vel(0);
  dbg_msg.cur_vel.y = s.cur_vel(1);
  dbg_msg.cur_vel.z = s.cur_vel(2);

  dbg_msg.cur_vel_body.x = s.cur_vel_body(0);
  dbg_msg.cur_vel_body.y = s.cur_vel_body(1);
  dbg_msg.cur_vel_body.z = s.cur_vel_body(2);

  dbg_msg.cur_Omega.x = s.cur_Omega(0);
  dbg_msg.cur_Omega.y = s.cur_Omega(1);
  dbg_msg.cur_Omega.z = s.cur_Omega(2);

  dbg_msg.cur_Omega_body.x = s.cur_Omega_body(0);
  dbg_msg.cur_Omega_body.y = s.cur_Omega_body(1);
  dbg_msg.cur_Omega_body.z = s.cur_Omega_body(2);

  dbg_msg.yaw.data = yaw;
  dbg_msg.pitch.data = pitch;
  dbg_msg.roll.data = roll;

  dbg_msg.yaw_d.data = yaw_d;
  dbg_msg.pitch_d.data = pitch_d;
  dbg_msg.roll_d.data = roll_d;

  dbg_msg.dot_yaw.data = dot_yaw;

  dbg_msg.theta_e.data = theta_e_tmp;

  dbg_msg.angular_velocity.data = omega_planned;
  dbg_msg.linear_velocity.data = v_planned;

  dbg_msg.thrust_cmd.data = thrust_cmd;
  dbg_msg.q_cmd.w = q_cmd.w();
  dbg_msg.q_cmd.x = q_cmd.x();
  dbg_msg.q_cmd.y = q_cmd.y();
  dbg_msg.q_cmd.z = q_cmd.z();

  dbg_msg.T_x.data = T(0);
  dbg_msg.T_y.data = T(1);

  dbg_msg.drag = drag;
  dbg_msg.accd = -K_T * e_v + dvx_dt;

  dbg_pub.publish(dbg_msg);
#endif // DBG

  return true; // return false means the algorithm is running 
}

void TrackFace::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

void TrackFace::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  s.cur_pos(0) = msg->pose.pose.position.x;
  s.cur_pos(1) = msg->pose.pose.position.y;
  s.cur_pos(2) = msg->pose.pose.position.z;

  s.cur_ori.w() = msg->pose.pose.orientation.w;
  s.cur_ori.x() = msg->pose.pose.orientation.x;
  s.cur_ori.y() = msg->pose.pose.orientation.y;
  s.cur_ori.z() = msg->pose.pose.orientation.z;

  s.cur_vel_body(0) = msg->twist.twist.linear.x;
  s.cur_vel_body(1) = msg->twist.twist.linear.y;
  s.cur_vel_body(2) = msg->twist.twist.linear.z;

  s.cur_vel = s.cur_ori * s.cur_vel_body;

  s.cur_Omega_body(0) = msg->twist.twist.angular.x;
  s.cur_Omega_body(1) = msg->twist.twist.angular.y;
  s.cur_Omega_body(2) = msg->twist.twist.angular.z;
  
  s.cur_Omega = s.cur_ori * s.cur_Omega_body;

  // publish marker for visualization
  if (start_tracking_)
    visualizer.visualizePassedTrajectory(s.cur_pos);

  // broadcast tf for visualization
  visualizer.visualizeFixedWing(s.cur_pos, s.cur_ori);
  transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                  msg->pose.pose.position.y,
                                  msg->pose.pose.position.z));
  transform.setRotation(tf::Quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world",
                                        "uav" + std::to_string(uav_id_)));

  has_odom_ = true;
}


