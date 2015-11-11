#ifndef RECORDED_TRAJECTORY_H
#define RECORDED_TRAJECTORY_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <sbc15_msgs/PlayAction.h>

class RecordedTrajectory : public State
{
public:
    TriggeredEvent event_done;
    TriggeredEvent event_failure;
public:
    RecordedTrajectory(State* parent, std::string trajectory);

    void entryAction();
    void iteration();

private:
//    std::string trajectory_;
//    actionlib::SimpleActionClient<sbc15_msgs::PlayAction> client_;
    bool started_;
    sbc15_msgs::PlayGoal goal_;

    void doneCb(const actionlib::SimpleClientGoalState& state,
           const sbc15_msgs::PlayResultConstPtr& result);
};

#endif // RECORDED_TRAJECTORY_H
