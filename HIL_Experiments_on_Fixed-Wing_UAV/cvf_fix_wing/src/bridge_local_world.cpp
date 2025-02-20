/*
 * @Author: “Shirley” “todo@163.com”
 * @Date: 2023-04-20 11:26:40
 * @LastEditors: “Shirley” shirleycoding@163.com
 * @LastEditTime: 2024-08-21 17:42:58
 * @FilePath: /src/planner_tracker/utils/localizations/src/bridgeLocalWorld.cpp
 * @Description: 把每个飞机的local下坐标转换到世界坐标下。
 *               「注意」目前只涉及平移，不涉及旋转。如果不同飞机的home坐标系不同，需要补充旋转，以及vel话题的旋转。
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
class BridgeLocalWorld
{
public:
    BridgeLocalWorld(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : n_(nh), pn_(pnh)
    {
        pn_.getParam("home_offset_x_", home_offset_x);
        pn_.getParam("home_offset_y_", home_offset_y);
        pn_.getParam("home_offset_z_", home_offset_z);
        pn_.param("home_offset_yaw_", home_offset_yaw, 0.0);

        pub1 = n_.advertise<geometry_msgs::PoseStamped>("world/local_position/pose", 1);
        pub2 = n_.advertise<nav_msgs::Odometry>("world/local_position/odom", 1);
        pub3 = n_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
        local_sub1 = n_.subscribe("mavros/local_position/pose", 1, &BridgeLocalWorld::sub1CB, this, ros::TransportHints().tcpNoDelay()); //no use, wait to change
        local_sub2 = n_.subscribe("mavros/local_position/odom", 1, &BridgeLocalWorld::sub2CB, this, ros::TransportHints().tcpNoDelay());
        world_sub3 = n_.subscribe("world/setpoint_position/local", 1, &BridgeLocalWorld::sub3CB, this, ros::TransportHints().tcpNoDelay());
    
        ori_init = Eigen::Quaterniond{cos(home_offset_yaw * 0.5), 0, 0, sin(home_offset_yaw * 0.5)};
        ori_init_inv = Eigen::Quaterniond{cos(-home_offset_yaw * 0.5), 0, 0, sin(-home_offset_yaw * 0.5)};
    }
    

private:
    ros::NodeHandle n_;
    ros::NodeHandle pn_;

    ros::Publisher pub1, pub2, pub3;
    ros::Subscriber local_sub1, local_sub2, world_sub3;

    geometry_msgs::PoseStamped pose;
    nav_msgs::Odometry odom;
    geometry_msgs::PoseStamped set_local;

    double home_offset_x, home_offset_y, home_offset_z;
    double home_offset_yaw; // NEU坐标系，左转为正 -pi～pi
    Eigen::Quaterniond ori_init, ori_init_inv;

    void sub1CB(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        pose = *msg;
        pose.pose.position.x += home_offset_x;
        pose.pose.position.y += home_offset_y;
        pose.pose.position.z += home_offset_z;

        Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        q = q * ori_init;
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();

        pub1.publish(pose);
    }

    void sub2CB(const nav_msgs::OdometryConstPtr msg)
    {
        odom = *msg;
        odom.pose.pose.position.x += home_offset_x;
        odom.pose.pose.position.y += home_offset_y;
        odom.pose.pose.position.z += home_offset_z;

        Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
        q = q * ori_init;
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        
        odom.header.frame_id = "world";
        pub2.publish(odom);
    }
    void sub3CB(const geometry_msgs::PoseStampedConstPtr msg)
    {
        set_local = *msg;
        set_local.pose.position.x -= home_offset_x;
        set_local.pose.position.y -= home_offset_y;
        set_local.pose.position.z -= home_offset_z;

        Eigen::Quaterniond q(set_local.pose.orientation.w, set_local.pose.orientation.x, set_local.pose.orientation.y, set_local.pose.orientation.z);
        q =  q * ori_init_inv;
        set_local.pose.orientation.w = q.w();
        set_local.pose.orientation.x = q.x();
        set_local.pose.orientation.y = q.y();
        set_local.pose.orientation.z = q.z();

        pub3.publish(set_local);
    }
}; // End of class BridgeLocalWorld

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_state_sender");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    BridgeLocalWorld bridge(nh, pnh);

    ros::spin();

    return 0;
}
