/// HEADER
#include "map_explorer.h"

/// COMPONENT
#include "../states/global_state.h"

/// SYSTEM
#include <nav_msgs/GetMap.h>
#include <opencv2/opencv.hpp>
#include <path_msgs/NavigateToGoalGoal.h>
#include <queue>

MapExplorer::MapExplorer()
    : exploring_(false)
{
    GlobalState& global = GlobalState::getInstance();

    std::string map_service = "/exploration/dynamic_map";
    global.private_nh.param("map_service",map_service, map_service);
    map_service_client = global.private_nh.serviceClient<nav_msgs::GetMap> (map_service);
//    map_service_client.waitForExistence();

    search_space_map_pub_ = global.private_nh.advertise<nav_msgs::OccupancyGrid>("search_space", 1);
}

void MapExplorer::startExploring()
{
    findExplorationPoint();
}

bool MapExplorer::isExploring()
{
    return exploring_;
}

void MapExplorer::stopExploring()
{
}


void MapExplorer::findExplorationPoint()
{
    GlobalState& global = GlobalState::getInstance();
    ros::Rate rate(2);

    nav_msgs::GetMap map_service;
    while(!map_service_client.call(map_service)) {
        ROS_ERROR("couldn't get map");
        ros::spinOnce();
        rate.sleep();
    }

    const nav_msgs::OccupancyGrid& map = map_service.response.map;

    tf::Vector3 own_pos = global.pose.getOrigin();
    cv::Point2i map_pos;
    map_pos.x = (own_pos.x() - map.info.origin.position.x) / map.info.resolution;
    map_pos.y = (own_pos.y() - map.info.origin.position.y) / map.info.resolution;

    splitMap(map, map_pos);

    cv::Mat debug(search_space.rows, search_space.cols, CV_8UC3);
    for(int row = 0; row < search_space.rows; ++row) {
        for(int col = 0; col < search_space.cols; ++col) {
            cv::Vec3b& pxl = debug.at<cv::Vec3b>(row, col);
            const uchar& cell = search_space.at<uchar>(row, col);
            pxl = cv::Vec3b::all(cell);
        }
    }
    double min_distance = 1 /*m*/ / map.info.resolution;
    cv::Point2i start = findNearestFreePoint(map_pos, debug);

    if(start == map_pos) {
        std::cout << "start at map_pos" << std::endl;
    }

    cv::Point2i poi = findPOI(start, tf::getYaw(global.pose.getRotation()), min_distance, debug);

    cv::circle(debug, poi, 5, cv::Scalar(0xFF, 0xCC, 0x00), CV_FILLED, CV_AA);
    cv::circle(debug, map_pos, 5, cv::Scalar(0x00, 0xCC, 0xFF), CV_FILLED, CV_AA);

    tf::Vector3 poi_pos;
    poi_pos.setX(poi.x * map.info.resolution + map.info.origin.position.x);
    poi_pos.setY(poi.y * map.info.resolution + map.info.origin.position.y);
    tf::Quaternion orientation = tf::createQuaternionFromYaw(std::atan2(poi.y - map_pos.y, poi.x - map_pos.x));

    tf::Pose target(orientation, poi_pos);
    global.moveTo(target, boost::bind(&MapExplorer::doneCb, this, _1, _2), path_msgs::NavigateToGoalGoal::FAILURE_MODE_ABORT);

    //    cv::imshow("search_space", search_space);
    //    cv::imshow("debug", debug);

    //    cv::waitKey(500);


    visualization_msgs::Marker marker = global.makeMarker(1,0,0, "exploration goal", 0);
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 2.0;
    marker.color.a = 0.4;
    marker.pose.position.x = poi_pos.x();
    marker.pose.position.y = poi_pos.y();
    marker.pose.position.z = marker.scale.z / 2.0;
    global.mark(marker);

    if(poi == map_pos) {
        std::cerr << "poi is own pose -> abort" << std::endl;
        exploring_ = false;
    } else {

        std::cerr << "got exploration point " << poi.x << " / " << poi.y << std::endl;
        exploring_ = true;
    }
}

namespace {
std::size_t index(const cv::Point2i& pt, std::size_t step) {
    return pt.y * step + pt.x;
}
std::size_t index(int x, int y, std::size_t step) {
    return y * step + x;
}
}

cv::Point2i MapExplorer::findNearestFreePoint(const cv::Point2i &start, cv::Mat &debug)
{
    int rows = search_space.rows;
    int cols = search_space.cols;
    std::size_t M = rows * cols;
    bool visited[M];
    for(std::size_t i = 0; i < M; ++i) {
        visited[i] = false;
    }
    visited[index(start, cols)] = true;

    std::deque<cv::Point2i> Q;
    Q.push_back(start);

    static const int neighbor_x[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int neighbor_y[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    while(!Q.empty()) {
        cv::Point2i current = Q.front();
        Q.pop_front();

        for(std::size_t n = 0; n < 8; ++n) {
            int nx = current.x + neighbor_x[n];
            int ny = current.y + neighbor_y[n];

            if(nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
                if(!visited[index(nx, ny, cols)]) {
                    visited[index(nx, ny, cols)] = true;
                    const uchar& cell = search_space.at<uchar>(ny, nx);

                    if(cell == UNKNOWN) {
                        Q.push_back(cv::Point2i(nx, ny));
                    } else if(cell == FREE) {
                        return cv::Point2i(nx, ny);
                    }
                }
            }
        }
    }

    return start;
}

cv::Point2i MapExplorer::findPOI(const cv::Point2i &start, double theta, double min_distance, cv::Mat& debug)
{
    int rows = search_space.rows;
    int cols = search_space.cols;
    std::size_t M = rows * cols;
    bool visited[M];
    for(std::size_t i = 0; i < M; ++i) {
        visited[i] = false;
    }
    visited[index(start, cols)] = true;

    std::deque<cv::Point2i> Q;
    Q.push_back(start);

    static const int neighbor_x[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int neighbor_y[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    cv::Point2i result = start;
    struct Point {
        cv::Point2i pt;
        double priority;

        bool operator < (const Point& rhs) const
        {
            return priority < rhs.priority;
        }
    };
    std::priority_queue<Point> candidates;

    while(!Q.empty()) {
        cv::Point2i current = Q.front();
        Q.pop_front();

        for(std::size_t n = 0; n < 8; ++n) {
            int nx = current.x + neighbor_x[n];
            int ny = current.y + neighbor_y[n];

            if(nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
                if(!visited[index(nx, ny, cols)]) {
                    visited[index(nx, ny, cols)] = true;
                    const uchar& cell = search_space.at<uchar>(ny, nx);
                    if(cell == UNKNOWN) {
                        double distance = hypot(nx-start.x, ny-start.y);
                        float dist_to_obstacles = distance_to_obstacle.at<float>(ny, nx);
                        if(distance > min_distance) {
                            //                            std::cerr << "found candidate cell " << nx << ", " << ny << " at distance" << distance << " m" << std::endl;
                            cv::Point2i candidate(nx, ny);
                            std::vector<cv::Point2i>::iterator it = std::find(blacklist_.begin(), blacklist_.end(), candidate);
                            if(it == blacklist_.end()) {
                                Point c;
                                c.pt = candidate;
                                c.priority = dist_to_obstacles;
                                candidates.push(c);
                            }
                        }
                        debug.at<cv::Vec3b>(ny, nx) = cv::Vec3b(0xFF, 0x00, 0x00);
                    } else if(cell != OBSTACLE) {
                        Q.push_back(cv::Point2i(nx, ny));
                        debug.at<cv::Vec3b>(ny, nx) = cv::Vec3b(0x00, 0x00, 0xFF);
                    }
                }
            }
        }
    }


    ROS_INFO_STREAM("got " << candidates.size() << " exploration candidates");


    if(!candidates.empty()) {
        result = candidates.top().pt;
        search_space.at<uchar>(result.y, result.x) = 240;
        candidates.pop();
        while(!candidates.empty()) {
            auto pt = candidates.top().pt;
            candidates.pop();
            search_space.at<uchar>(pt.y, pt.x) = 180;

        }

        blacklist_.push_back(result);
    } else {
        blacklist_.clear();
    }


    // PUBLISH SEARCH SPACE AS GRID MAP

    int w = last_map.info.width;
    int h = last_map.info.height;

    nav_msgs::OccupancyGrid ss;
    ss.header = last_map.header;
    ss.info = last_map.info;
    ss.data.resize(w*h, 0);

    int i = 0;
    for(int row = 0; row < h; ++row) {
        for(int col = 0; col < w; ++col) {
            const uchar& val = search_space.at<uchar>(row, col);
            if(val == 255) {
                ss.data[i++] = val;
            } else {
                // val is in [0,254]
                ss.data[i++] = (val / 254.0) * 100.0;
            }
        }
    }

    search_space_map_pub_.publish(ss);

    return result;
}

void MapExplorer::splitMap(const nav_msgs::OccupancyGrid &map, cv::Point2i map_pos)
{
    last_map = map;

    int w = map.info.width;
    int h = map.info.height;

    const cv::Mat data(h, w, CV_8SC1, const_cast<signed char*>(map.data.data()));

    cv::Mat map_free(h, w, CV_8UC1, cv::Scalar::all(255));
    cv::Mat map_obstacle(h, w, CV_8UC1, cv::Scalar::all(255));
    cv::Mat map_unknown(h, w, CV_8UC1, cv::Scalar::all(255));

    for(int row = 0; row < h; ++row) {
        for(int col = 0; col < w; ++col) {
            const char& cell = data.at<char>(row, col);

            bool free = cell >= 0 && cell <= 10;
            bool occupied = cell > 10;

            double distance = std::hypot(row - map_pos.y, col - map_pos.x);
            if(distance < 10.0) {
                free = true;
                occupied = false;
            }

            if(free) {
                map_free.at<uchar>(row, col) = 0;
            } else if(occupied) {
                map_obstacle.at<uchar>(row, col) = 0;
            } else {
                map_unknown.at<uchar>(row, col) = 0;
            }
        }
    }


    int kernel_size = 4;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size( 2*kernel_size + 1, 2*kernel_size+1 ),
                                               cv::Point( kernel_size, kernel_size ) );
    int small_kernel_size = 3;
    cv::Mat small_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                     cv::Size( 2*small_kernel_size + 1, 2*small_kernel_size+1 ),
                                                     cv::Point( small_kernel_size, small_kernel_size ) );

    //    cv::dilate(map_free, map_free_safe, kernel);


    cv::dilate(map_free, map_free, kernel);
    cv::erode(map_obstacle, map_obstacle, kernel);
    cv::erode(map_unknown, map_unknown, small_kernel);

    cv::distanceTransform(map_obstacle, distance_to_obstacle, CV_DIST_L2, 5);
    assert(distance_to_obstacle.type() == CV_32FC1);

    search_space = cv::Mat(h, w, CV_8UC1, cv::Scalar::all(0));

    for(int row = 0; row < h; ++row) {
        for(int col = 0; col < w; ++col) {
            char& e = search_space.at<char>(row, col);

            bool free = 0 == map_free.at<char>(row, col);
            bool obstacle = 0 == map_obstacle.at<char>(row, col);
            bool unknown = 0 == map_unknown.at<char>(row, col);

            if(obstacle) {
                e = OBSTACLE;
            } else if(unknown) {
                e = UNKNOWN;
            } else {
                const float& dist_to_obst = distance_to_obstacle.at<float>(row, col);
                e = (254 - std::min(254.f, dist_to_obst * 10.0f));
            }
        }
    }
}

void MapExplorer::doneCb(const actionlib::SimpleClientGoalState& /*state*/,
                         const path_msgs::NavigateToGoalResultConstPtr& /*result*/)
{
    ROS_WARN("done exploring");
    exploring_ = false;
}
