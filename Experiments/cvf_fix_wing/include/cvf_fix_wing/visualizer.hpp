#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

// Visualizer for the planner
class Visualizer
{
private:
    // config contains the scale for some markers
    ros::NodeHandle nh;

    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    ros::Publisher routePub;
    ros::Publisher routeWayPointsPub;
    ros::Publisher wayPointsPub;
    ros::Publisher spherePub;
    ros::Publisher spherePub2;
    ros::Publisher spherePub3;
    ros::Publisher passedTrajectoryPub;
    ros::Publisher desiredPassedTrajectoryPub;
    ros::Publisher arrowPub1;
    ros::Publisher arrowPub2;

    std::string fixedwing_mesh_source;
    double fixedwing_color_r{1.0};
    double fixedwing_color_g{0.1};
    double fixedwing_color_b{0.1};
    visualization_msgs::Marker passedTrajMarker, desiredPassedTrajMarker;

    // visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker;
    // visualization_msgs::Marker meshMarker, edgeMarker;

public:
    ros::Publisher speedPub;
    ros::Publisher thrPub;
    ros::Publisher tiltPub;
    ros::Publisher pitchPub;
    ros::Publisher rollPub;
    ros::Publisher bdrPub;
    ros::Publisher fixedwingPub;

public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        routePub = nh.advertise<visualization_msgs::Marker>("visualizer/route", 10);
        routeWayPointsPub = nh.advertise<visualization_msgs::Marker>("visualizer/routeWayPoints", 10);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("visualizer/waypoints", 10);
        spherePub = nh.advertise<visualization_msgs::Marker>("visualizer/spheres", 1000);
        spherePub2 = nh.advertise<visualization_msgs::Marker>("visualizer/spheres2", 1000);
        spherePub3 = nh.advertise<visualization_msgs::Marker>("visualizer/spheres3", 1000);
        fixedwingPub = nh.advertise<visualization_msgs::Marker>("visualizer/fixedwing", 1);
        arrowPub1 = nh.advertise<visualization_msgs::Marker>("visualizer/arrow1", 1);
        arrowPub2 = nh.advertise<visualization_msgs::Marker>("visualizer/arrow2", 1);
        passedTrajectoryPub = nh.advertise<visualization_msgs::Marker>("visualizer/passed_trajectory", 1);
        desiredPassedTrajectoryPub = nh.advertise<visualization_msgs::Marker>("visualizer/desired_passed_trajectory", 1);
        speedPub = nh.advertise<std_msgs::Float64>("visualizer/speed", 1000);
        thrPub = nh.advertise<std_msgs::Float64>("visualizer/total_thrust", 1000);
        tiltPub = nh.advertise<std_msgs::Float64>("visualizer/tilt_angle", 1000);
        pitchPub = nh.advertise<std_msgs::Float64>("visualizer/pitch_angle", 1000);
        rollPub = nh.advertise<std_msgs::Float64>("visualizer/roll_angle", 1000);
        bdrPub = nh.advertise<std_msgs::Float64>("visualizer/body_rate", 1000);

        passedTrajMarker.type = visualization_msgs::Marker::LINE_STRIP;
        passedTrajMarker.header.frame_id = "world";
        passedTrajMarker.action = visualization_msgs::Marker::ADD;
        passedTrajMarker.id = 0;
        passedTrajMarker.ns = "passed_trajectory";
        // 黄绿色 paprus
        passedTrajMarker.color.r = 0.60; // 153 99
        passedTrajMarker.color.g = 0.62; // 158 9e
        passedTrajMarker.color.b = 0.11; // 28  1c
        // orange creamsicle
        passedTrajMarker.color.r = 1.0;  // ff
        passedTrajMarker.color.g = 0.72; // b7
        passedTrajMarker.color.b = 0.06; // 10
        passedTrajMarker.color.a = 0.70;
        passedTrajMarker.scale.x = 3.00;

        desiredPassedTrajMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        desiredPassedTrajMarker.header.frame_id = "world";
        desiredPassedTrajMarker.action = visualization_msgs::Marker::ADD;
        desiredPassedTrajMarker.id = 0;
        desiredPassedTrajMarker.ns = "desired_passed_trajectory";
        // Whimsical blue
        desiredPassedTrajMarker.color.r = 0.00; // 00
        desiredPassedTrajMarker.color.g = 0.92; // eb
        desiredPassedTrajMarker.color.b = 1.00; // ff
        desiredPassedTrajMarker.color.a = 0.70;
        desiredPassedTrajMarker.scale.x = 3.00;
    }
    void setFixedWingMeshSource(const std::string &filename)
    {
        fixedwing_mesh_source = filename;
    }
    void setFixedWingColor(const double &r, const double &g, const double &b)
    {
        fixedwing_color_r = r;
        fixedwing_color_g = g;
        fixedwing_color_b = b;
    }

    // Visualize passed trajectory
    inline void visualizePassedTrajectory(const Eigen::Vector3d &pos)
    {
        passedTrajMarker.header.stamp = ros::Time::now();
        geometry_msgs::Point point;
        point.x = pos(0);
        point.y = pos(1);
        point.z = pos(2);
        passedTrajMarker.points.push_back(point);
        passedTrajectoryPub.publish(passedTrajMarker);
    }
    // Visualize passed trajectory
    inline void visualizeDesiredPassedTrajectory(const Eigen::Vector3d &pos)
    {
        desiredPassedTrajMarker.header.stamp = ros::Time::now();
        geometry_msgs::Point point;
        point.x = pos(0);
        point.y = pos(1);
        point.z = pos(2);
        desiredPassedTrajMarker.points.push_back(point);
        desiredPassedTrajectoryPub.publish(desiredPassedTrajMarker);
    }
    inline void visualizeArrow1(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const double scale)
    { // inputs: p start point; v direction vector
        visualization_msgs::Marker arrow_mark;
        arrow_mark.header.frame_id = "world";
        arrow_mark.header.stamp = ros::Time::now();
        arrow_mark.ns = "arrow";
        arrow_mark.id = 0;
        arrow_mark.type = visualization_msgs::Marker::LINE_LIST;
        arrow_mark.action = visualization_msgs::Marker::ADD;
        arrow_mark.scale.x = 3.0;
        arrow_mark.scale.y = 3.0;
        arrow_mark.scale.z = 3.0;
        arrow_mark.color.a = 1.0;
        arrow_mark.color.r = 1.0;
        arrow_mark.color.g = 0.1;
        arrow_mark.color.b = 0.1;

        geometry_msgs::Point point;
        point.x = p(0);
        point.y = p(1);
        point.z = p(2);
        arrow_mark.points.push_back(point);
        point.x = p(0) + v(0) * scale;
        point.y = p(1) + v(1) * scale;
        point.z = p(2) + v(2) * scale;
        arrow_mark.points.push_back(point);
        arrowPub1.publish(arrow_mark);
    }
    inline void visualizeArrow2(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const double scale)
    { // inputs: p start point; v direction vector
        visualization_msgs::Marker arrow_mark;
        arrow_mark.header.frame_id = "world";
        arrow_mark.header.stamp = ros::Time::now();
        arrow_mark.ns = "arrow";
        arrow_mark.id = 0;
        arrow_mark.type = visualization_msgs::Marker::LINE_LIST;
        arrow_mark.action = visualization_msgs::Marker::ADD;
        arrow_mark.scale.x = 3.0;
        arrow_mark.scale.y = 3.0;
        arrow_mark.scale.z = 3.0;
        arrow_mark.color.a = 1.0;
        arrow_mark.color.r = 0.1;
        arrow_mark.color.g = 0.5;
        arrow_mark.color.b = 0.8;

        geometry_msgs::Point point;
        point.x = p(0);
        point.y = p(1);
        point.z = p(2);
        arrow_mark.points.push_back(point);
        point.x = p(0) + v(0) * scale;
        point.y = p(1) + v(1) * scale;
        point.z = p(2) + v(2) * scale;
        arrow_mark.points.push_back(point);
        arrowPub2.publish(arrow_mark);
    }
    // current fixed wing model
    inline void visualizeFixedWing(const Eigen::Vector3d &p, const Eigen::Quaterniond &q)
    {
        visualization_msgs::Marker fixedwing_mesh;
        fixedwing_mesh.header.frame_id = "world";
        fixedwing_mesh.header.stamp = ros::Time::now();
        fixedwing_mesh.ns = "mesh";
        fixedwing_mesh.id = 0;
        fixedwing_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
        fixedwing_mesh.mesh_resource = fixedwing_mesh_source;
        fixedwing_mesh.action = visualization_msgs::Marker::ADD;
        fixedwing_mesh.pose.position.x = p(0);
        fixedwing_mesh.pose.position.y = p(1);
        fixedwing_mesh.pose.position.z = p(2);
        fixedwing_mesh.pose.orientation.w = q.w();
        fixedwing_mesh.pose.orientation.x = q.x();
        fixedwing_mesh.pose.orientation.y = q.y();
        fixedwing_mesh.pose.orientation.z = q.z();
        fixedwing_mesh.scale.x = 0.1;
        fixedwing_mesh.scale.y = 0.1;
        fixedwing_mesh.scale.z = 0.1;
        fixedwing_mesh.color.a = 1.0;
        fixedwing_mesh.color.r = fixedwing_color_r;
        fixedwing_mesh.color.g = fixedwing_color_g;
        fixedwing_mesh.color.b = fixedwing_color_b;
        fixedwingPub.publish(fixedwing_mesh);
    }
    // current position
    inline void visualizeSphere3(const Eigen::Vector3d &center,
                                 const double &radius)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "world";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.50;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);
        spherePub3.publish(sphereDeleter);
        spherePub3.publish(sphereMarkers);
    }
    // current position
    inline void visualizeSphere2(const Eigen::Vector3d &center,
                                 const double &radius)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "world";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.50;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);
        spherePub2.publish(sphereDeleter);
        spherePub2.publish(sphereMarkers);
    }
    // Visualize all spheres with centers sphs and the same radius
    inline void visualizeSphere(const Eigen::Vector3d &center,
                                const double &radius)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "world";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        spherePub.publish(sphereDeleter);
        spherePub.publish(sphereMarkers);
    }

public:
    typedef std::shared_ptr<Visualizer> Ptr;
};

#endif