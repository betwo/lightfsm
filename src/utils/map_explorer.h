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

protected:
    nav_msgs::OccupancyGridPtr generateSearchSpace(const cv::Point2i& map_pos, double min_move_distance, double min_distance_to_obstacles, double max_distance_to_unknown);
    
    void searchDirectionCallback(const geometry_msgs::PointConstPtr& point);

private:
    void findExplorationPoint();
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const path_msgs::NavigateToGoalResultConstPtr& result);

    void splitMap(const nav_msgs::OccupancyGrid& map, cv::Point2i map_pos);
    cv::Point2i findNearestFreePoint(const cv::Point2i& start);
    cv::Point2i findPOI(const cv::Point2i& start, double min_distance);

private:
    enum STATES {
        OBSTACLE = 254,
        FREE = 0,
        UNKNOWN = 255
    };

private:
    bool exploring_;

    ros::ServiceClient map_service_client;

    bool last_planning_failed_;
    std::string planner_;
    nav_msgs::OccupancyGrid last_map;

    cv::Mat search_space;
    cv::Mat distance_to_obstacle;
    cv::Mat distance_to_unknown;

    bool has_search_dir_;
    geometry_msgs::Point search_dir_;

    ros::Subscriber search_dir_sub_;
    ros::Publisher search_space_map_pub_;
    ros::Publisher map_pub_;

    std::vector<cv::Point2i> blacklist_;
};

#endif // MAP_EXPLORER_H
