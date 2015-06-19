/// HEADER
#include "map_explorer.h"

/// COMPONENT
#include "../states/global_state.h"

/// SYSTEM
#include <nav_msgs/GetMap.h>
#include <opencv2/opencv.hpp>

MapExplorer::MapExplorer()
    : exploring_(false)
{
    GlobalState& global = GlobalState::getInstance();

    std::string map_service = "/exploration/dynamic_map";
    global.nh.param("map_service",map_service, map_service);
    map_service_client = global.nh.serviceClient<nav_msgs::GetMap> (map_service);
    map_service_client.waitForExistence();

    search_space_map_pub_ = global.nh.advertise<nav_msgs::OccupancyGrid>("search_space", 1);
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
    splitMap(map);

    cv::Mat debug(search_space.rows, search_space.cols, CV_8UC3);
    for(int row = 0; row < search_space.rows; ++row) {
        for(int col = 0; col < search_space.cols; ++col) {
            cv::Vec3b& pxl = debug.at<cv::Vec3b>(row, col);
            const uchar& cell = search_space.at<uchar>(row, col);
            pxl = cv::Vec3b::all(cell);
        }
    }

    tf::Vector3 own_pos = global.pose.getOrigin();
    cv::Point2i map_pos;
    map_pos.x = (own_pos.x() - map.info.origin.position.x) / map.info.resolution;
    map_pos.y = (own_pos.y() - map.info.origin.position.y) / map.info.resolution;

    double min_distance = 10 /*m*/ / map.info.resolution;
    cv::Point2i start = findNearestFreePoint(search_space, map_pos, debug);

    if(start == map_pos) {
        std::cout << "start at map_pos" << std::endl;
    }

    cv::Point2i poi = findPOI(search_space, start, min_distance, debug);

    cv::circle(debug, poi, 5, cv::Scalar(0xFF, 0xCC, 0x00), CV_FILLED, CV_AA);
    cv::circle(debug, map_pos, 5, cv::Scalar(0x00, 0xCC, 0xFF), CV_FILLED, CV_AA);

    tf::Vector3 poi_pos;
    poi_pos.setX(poi.x * map.info.resolution + map.info.origin.position.x);
    poi_pos.setY(poi.y * map.info.resolution + map.info.origin.position.y);
    tf::Quaternion orientation = tf::createQuaternionFromYaw(std::atan2(poi.y - map_pos.y, poi.x - map_pos.x));

    tf::Pose target(orientation, poi_pos);
    global.moveTo(target, boost::bind(&MapExplorer::doneCb, this, _1, _2));

//    cv::imshow("search_space", search_space);
//    cv::imshow("debug", debug);

//    cv::waitKey(500);


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

cv::Point2i MapExplorer::findNearestFreePoint(const cv::Mat &search_space, const cv::Point2i &start, cv::Mat &debug)
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

cv::Point2i MapExplorer::findPOI(const cv::Mat &search_space, const cv::Point2i &start, double min_distance, cv::Mat& debug)
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
                        double distance = hypot(nx-start.x, ny-start.y);
                        if(distance > min_distance) {
                            std::cerr << "found candidate cell " << nx << ", " << ny << " at distance" << distance << " m" << std::endl;
                            cv::Point2i candidate(nx, ny);
//                            std::vector<cv::Point2i>::iterator it = std::find(blacklist_.begin(), blacklist_.end(), candidate);
//                            if(it == blacklist_.end()) {
//                                blacklist_.push_back(candidate);


                                return candidate;
//                            }
                        } else {
//                            std::cerr << "ignore unknown cell " << nx << ", " << ny << " because it is only " << distance << " m away (min is " << min_distance << ")" << std::endl;
                        }
                        debug.at<cv::Vec3b>(ny, nx) = cv::Vec3b(0xFF, 0x00, 0x00);
                    } else if(cell == FREE) {
                        Q.push_back(cv::Point2i(nx, ny));
                        debug.at<cv::Vec3b>(ny, nx) = cv::Vec3b(0x00, 0x00, 0xFF);
                    }
                }
            }
        }
    }

    return start;
}

void MapExplorer::splitMap(const nav_msgs::OccupancyGrid &map)
{
    int w = map.info.width;
    int h = map.info.height;

    search_space_map_pub_.publish(map);

    const cv::Mat data(h, w, CV_8SC1, const_cast<signed char*>(map.data.data()));

    cv::Mat map_free(h, w, CV_8UC1, cv::Scalar::all(UNKNOWN));
    cv::Mat map_obstacle(h, w, CV_8UC1, cv::Scalar::all(UNKNOWN));

    for(int row = 0; row < h; ++row) {
        for(int col = 0; col < w; ++col) {
            const char& cell = data.at<char>(row, col);
            if(cell >= 0 && cell <= 10) {
                map_free.at<uchar>(row, col) = FREE;
            } else if(cell > 10) {
                map_obstacle.at<uchar>(row, col) = OBSTACLE;
            }
        }
    }


    int kernel_size = 5;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size( 2*kernel_size + 1, 2*kernel_size+1 ),
                                               cv::Point( kernel_size, kernel_size ) );

//    cv::dilate(map_free, map_free_safe, kernel);

    cv::erode(map_free, map_free, kernel);
    cv::erode(map_obstacle, map_obstacle, kernel);

    search_space = cv::min(map_obstacle, map_free);
}

void MapExplorer::doneCb(const actionlib::SimpleClientGoalState& /*state*/,
                         const path_msgs::NavigateToGoalResultConstPtr& /*result*/)
{
    exploring_ = false;
}
