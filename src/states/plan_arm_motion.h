#ifndef PLAN_ARM_MOTION_H
#define PLAN_ARM_MOTION_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
//#include <actionlib/client/simple_action_client.h>
#include <sbc15_msgs/CupGrippAction.h>
#include "ros/ros.h"

class PlanArmMotion: public State
{
public:
    TriggeredEvent event_at_goal;
    TriggeredEvent event_failure;
public:
    PlanArmMotion(State* parent, int retries);

    void entryAction();
    void iteration();

private:
//    actionlib::SimpleActionClient<sbc15_msgs::CupGrippAction> client_;
    ros::ServiceClient planedTrajectoryClient_;
    int retries_;
    int retries_left_;
    bool started_;
};

#endif // PLAN_ARM_MOTION_H
