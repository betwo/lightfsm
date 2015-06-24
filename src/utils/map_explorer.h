#ifndef MAP_EXPLORER_H
#define MAP_EXPLORER_H

/// INCLUDE ROS STUFF AND IGNORE WARNINGS FROM THERE ///
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <path_msgs/NavigateToGoalAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/LinearMath/Vector3.h>
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
/////////////////////////////////////////////////////////


class MapExplorer
{
public:
    MapExplorer();

    void startExploring();
    void stopExploring();

    bool isExploring();

private:
    void findExplorationPoint();
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const path_msgs::NavigateToGoalResultConstPtr& result);

    void splitMap(const nav_msgs::OccupancyGrid& map, cv::Point2i map_pos);
    cv::Point2i findNearestFreePoint(const cv::Point2i& start, cv::Mat &debug);
    cv::Point2i findPOI(const cv::Point2i& start, double theta, double min_distance, cv::Mat &debug);

private:
    enum STATES {
        OBSTACLE = 254,
        FREE = 0,
        UNKNOWN = 255
    };

private:
    bool exploring_;

    ros::ServiceClient map_service_client;

    nav_msgs::OccupancyGrid last_map;

    cv::Mat search_space;
    cv::Mat distance_to_obstacle;

    ros::Publisher search_space_map_pub_;

    std::vector<cv::Point2i> blacklist_;
};

#endif // MAP_EXPLORER_H
