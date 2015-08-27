#ifndef PLAN_ARM_MOTION_H
#define PLAN_ARM_MOTION_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
#include <actionlib/client/simple_action_client.h>
#include <sbc15_msgs/CupGrippAction.h>

class PlanArmMotion: public State
{
public:
    TriggeredEvent event_at_goal;
    TriggeredEvent event_failure;
public:
    PlanArmMotion(State* parent);

    void entryAction();
    void iteration();

private:
    actionlib::SimpleActionClient<sbc15_msgs::CupGrippAction> client_;
};

#endif // PLAN_ARM_MOTION_H
