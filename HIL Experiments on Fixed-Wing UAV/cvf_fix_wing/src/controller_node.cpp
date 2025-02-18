/*
 * @Author: “Shirley” shirleycoding@163.com
 * @Date: 2024-10-11 19:53:32
 * @LastEditors: “Shirley” shirleycoding@163.com
 * @LastEditTime: 2024-10-11 19:53:43
 * @FilePath: src/sacontroller.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "cvf_fix_wing/track_face.h"
int main(int argc, char **argv)
{
  ROS_INFO("SAController running...");
  ros::init(argc, argv, "SAController");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  TrackFace controller(nh, pnh);

  ros::spin();

  return 0;
}