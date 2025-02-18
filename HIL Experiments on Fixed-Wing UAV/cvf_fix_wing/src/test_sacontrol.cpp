#include "cvf_fix_wing/track_face.h"
#include "cvf_fix_wing/sacontrol_utils.h"

#include <iostream>

int main(int argc, char **argv)
{
  ROS_INFO("SAController running...");
  ros::init(argc, argv, "SAController");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  TrackFace controller(nh, pnh);

  struct curState s;
  s.cur_pos = Eigen::Vector3d(-60, 50, 30);
  s.cur_ori = Eigen::Quaterniond(1,1,1,1).normalized();  
  s.cur_vel_body = Eigen::Vector3d(10, 10, 0);

  s.cur_vel = s.cur_ori * s.cur_vel_body;
  s.cur_Omega_body = Eigen::Vector3d(0.1, 0.1, 0.1);
  s.cur_Omega = s.cur_ori * s.cur_Omega_body;

  double des_height = 25.0;
  Eigen::Vector2d des_p_2d(50, 50);
  double des_angle = M_PI / 2;
  double rho = 10.0;


  // 打印参数
  std::cout << "Current Position: " << s.cur_pos.transpose() << std::endl;
  std::cout << "Current Orientation (Quaternion): " << s.cur_ori.coeffs().transpose() << std::endl;
  std::cout << "Current Body Velocity: " << s.cur_vel_body.transpose() << std::endl;
  std::cout << "Current Inertial Velocity: " << s.cur_vel.transpose() << std::endl;
  std::cout << "Current Body Angular Velocity: " << s.cur_Omega_body.transpose() << std::endl;
  std::cout << "Current Inertial Angular Velocity: " << s.cur_Omega.transpose() << std::endl;



  Eigen::Quaterniond q_cmd;
  double thrust_cmd = 0.0;
  controller.SAControl(s, des_p_2d, des_angle, des_height, rho, q_cmd, thrust_cmd);

  // 打印结果
  std::cout << "Desired Orientation (Quaternion): " << q_cmd.coeffs().transpose() << std::endl;
  std::cout << "Thrust Command: " << thrust_cmd << std::endl;

  Eigen::Matrix3d rot_mat_d;
  rot_mat_d = q_cmd.matrix();
  // 输出旋转矩阵
  std::cout << "Desired Rotation Matrix: " << std::endl;
  std::cout << rot_mat_d << std::endl;

  return 0;
}