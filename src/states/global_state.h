#ifndef GLOBAL_STATE_H
#define GLOBAL_STATE_H

/// SYSTEM
#include <boost/noncopyable.hpp>

/// INCLUDE ROS STUFF AND IGNORE WARNINGS FROM THERE ///
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <path_msgs/NavigateToGoalAction.h>
#include <sbc15_msgs/Object.h>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#pragma GCC diagnostic pop
/////////////////////////////////////////////////////////

class GlobalState : private boost::noncopyable
{
public:
    static GlobalState& getInstance();

private:
    GlobalState();

public:
    void talk(const std::string& text);
    void sound(int sound);

    tf::Transform getTransform(const std::string& from, const std::string& to, const ros::Time& time = ros::Time(0));

    void move(const geometry_msgs::Twist& twist);
    void moveUnsafe(const geometry_msgs::Twist& twist);
    void stopMoving();

    void moveTo(const tf::Pose& pose,
                boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb);
    void moveTo(const tf::Pose& pose,
                boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> feedbackCb);

    void moveTo(const tf::Pose& pose,
                double velocity,
                boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb);
    void moveTo(const tf::Pose& pose,
                double velocity,
                boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> feedbackCb);

    void moveTo(const geometry_msgs::PoseStamped& pose,
                boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb);
    void moveTo(const geometry_msgs::PoseStamped& pose,
                boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> feedbackCb);

    void moveTo(const geometry_msgs::PoseStamped& pose,
                double velocity,
                boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb);
    void moveTo(const geometry_msgs::PoseStamped& pose,
                double velocity,
                boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> feedbackCb);

    void update();
    void mark(const visualization_msgs::Marker& marker);

    static visualization_msgs::Marker makeMarker(float r, float g, float b, const std::string &ns, int id);

    std::vector<sbc15_msgs::Object> getObjects();

    void setSystemEnabled(const std::string& name, bool enabled);
    void sendSystemCommand(const std::string& name, const std::string& command);

private:
    void activeCb();
    void feedbackCb(const path_msgs::NavigateToGoalFeedbackConstPtr& feedback);

public:
    ros::NodeHandle nh;
    tf::Pose pose;

private:
//    ros::Publisher pub_speech_;
    ros::Publisher pub_move_;
    ros::Publisher pub_move_unsafe_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_sound_;

    std::map<std::string, ros::Publisher> pubs_systems_;

    actionlib::SimpleActionClient<path_msgs::NavigateToGoalAction> client_;

    ros::ServiceClient client_objects_;

    tf::TransformListener tfl_;

    double desired_speed_;
};

#endif // GLOBAL_STATE_H
