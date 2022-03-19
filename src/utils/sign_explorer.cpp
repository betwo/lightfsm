/// HEADER
#include "sign_explorer.h"

/// COMPONENT
#include "../states/global_state.h"

/// SYSTEM
#include <nav_msgs/GetMap.h>

SignExplorer::SignExplorer()
{
    GlobalState& global = GlobalState::getInstance();

    std::string map_service = "/dynamic_map";
    global.nh.param("map_service", map_service, map_service);
    map_service_client = global.nh.serviceClient<nav_msgs::GetMap>(map_service);
    map_service_client.waitForExistence();

    lookat_pt_cmd_publisher = global.nh.advertise<std_msgs::String>("/look_at/cmd", 10, true);
    sub_sign_ = global.nh.subscribe<sick_msgs::Sign>(
        "/signs", 10, std::bind(&SignExplorer::signCallback, this, std::placeholders::_1));

    angle_offset_ = 0.0;
}

void SignExplorer::startExploring()
{
    findExplorationPoint();
}

void SignExplorer::continueExploring()
{
    GlobalState& global = GlobalState::getInstance();

    tf::Vector3 own_pos = global.pose.getOrigin();
    tf::Vector3 center;
    tf::pointMsgToTF(global.map_classification_.center.position, center);

    tf::Vector3 diff = (own_pos - center);
    double angle = std::atan2(diff.y(), diff.x());
    angle += M_PI / 3.0;

    tf::Vector3 dir(std::cos(angle), std::sin(angle), 0);
    // tf::Vector3 dir = diff / diff.length();
    tf::Vector3 offset = 2 * global.map_classification_.outer_radius * dir;

    tf::Vector3 nearest_circle_pose = center + offset;

    global.lookat(nearest_circle_pose, "/map");
}

void SignExplorer::stopExploring()
{
    // reset view mode to look ahead
    std_msgs::String reset;
    reset.data = "reset";
    lookat_pt_cmd_publisher.publish(reset);
}

void SignExplorer::findExplorationPoint()
{
    GlobalState& global = GlobalState::getInstance();
    ros::Rate rate(2);

    nav_msgs::GetMap map_service;
    while (!map_service_client.call(map_service)) {
        ROS_ERROR_STREAM("couldn't get map");
        ros::spinOnce();
        rate.sleep();
    }
    //    const nav_msgs::OccupancyGrid& map = map_service.response.map;

    while (!global.map_classification_.valid) {
        ROS_ERROR("waiting for valid map classification");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("select target point on the circle");
    tf::Vector3 own_pos = global.pose.getOrigin();
    tf::Vector3 center;
    tf::pointMsgToTF(global.map_classification_.center.position, center);

    tf::Vector3 diff = (own_pos - center);

    double view = std::atan2(diff.y(), diff.x());
    double angle = view + M_PI / 2.0 + angle_offset_;

    double r = (global.map_classification_.inner_radius + global.map_classification_.outer_radius) / 2.0;
    tf::Vector3 circle_point = center + tf::Vector3(std::cos(angle) * r, std::sin(angle) * r, 0);
    tf::Quaternion orientation = tf::createQuaternionFromYaw(angle);

    tf::Pose target(orientation, circle_point);
    global.moveTo(target, 1.0, std::bind(&SignExplorer::doneCb, this, std::placeholders::_1, std::placeholders::_2));
}

void SignExplorer::doneCb(const actionlib::SimpleClientGoalState& /*state*/,
                          const path_msgs::NavigateToGoalResultConstPtr& result)
{
    if (result->status == path_msgs::NavigateToGoalResult::STATUS_SUCCESS) {
        angle_offset_ = 0.0;
    } else {
        angle_offset_ += M_PI / 8.0;
    }
    findExplorationPoint();
}

void SignExplorer::signCallback(const sick_msgs::SignConstPtr& sign)
{
}
