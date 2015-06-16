#ifndef FOLLOW_PATH_H
#define FOLLOW_PATH_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

/// SYSTEM
#include <ros/subscriber.h>
#include <deque>
#include <tf/tf.h>

/// INCLUDE ROS STUFF AND IGNORE WARNINGS FROM THERE ///
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <path_msgs/NavigateToGoalAction.h>
#include <path_msgs/NavigateToGoalFeedback.h>
#include <actionlib/client/simple_action_client.h>
#pragma GCC diagnostic pop
/////////////////////////////////////////////////////////

class FollowPath : public State
{
public:
    TriggeredEvent event_positioned;
    TriggeredEvent event_error;

public:
    FollowPath(State* parent, int retries);

    void entryAction();
    void iteration();


    void moveTo(const tf::Pose& pose, TriggeredEvent &event);
    void moveTo(const tf::Pose& pose, TriggeredEvent &event, boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)>);
    void moveTo(const geometry_msgs::PoseStamped& pose, TriggeredEvent &event);
    void moveTo(const geometry_msgs::PoseStamped& pose, TriggeredEvent &event, boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)>);

private:
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const path_msgs::NavigateToGoalResultConstPtr& result);

    int retries_;
    int retries_left_;

    geometry_msgs::PoseStamped goal_;
};



#endif // FOLLOW_PATH_H
