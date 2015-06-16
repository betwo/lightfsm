#ifndef SIGN_EXPLORER_H
#define SIGN_EXPLORER_H

/// INCLUDE ROS STUFF AND IGNORE WARNINGS FROM THERE ///
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <path_msgs/NavigateToGoalAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sick_msgs/Sign.h>
#include <tf/tf.h>
#pragma GCC diagnostic pop
/////////////////////////////////////////////////////////

class SignExplorer
{
public:
    SignExplorer();

    void startExploring();
    void continueExploring();
    void stopExploring();

private:
    void findExplorationPoint();
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const path_msgs::NavigateToGoalResultConstPtr& result);
    void signCallback(const sick_msgs::SignConstPtr &sign);

private:
    ros::ServiceClient map_service_client;

    ros::Publisher lookat_pt_cmd_publisher;
    ros::Subscriber sub_sign_;

    double angle_offset_;
};

#endif // SIGN_EXPLORER_H
