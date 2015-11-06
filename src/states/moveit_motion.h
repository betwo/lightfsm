#ifndef MOVEITMOTION_H
#define MOVEITMOTION_H
/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
#include "global_state.h"

#include <actionlib/client/simple_action_client.h>
#include <sbc15_msgs/MoveManipulatorAction.h>

class MoveitMotion: public State
{
public:
    TriggeredEvent event_done;
    TriggeredEvent event_timeout;
    TriggeredEvent event_failure;
    TriggeredEvent event_planning_failed ;
    TriggeredEvent event_servo_control_failed;


public:
    MoveitMotion(State* parent, int retries);
    MoveitMotion(State* parent, int retries, const ArmGoal& goal);

    void entryAction();
    void iteration();

private:
    int retries_;
    int retries_left_;
    bool started_;
    bool takeGlobalStateGoal_;

    sbc15_msgs::MoveManipulatorGoal goal_;
    actionlib::SimpleActionClient<sbc15_msgs::MoveManipulatorAction> client_;
    ArmGoal constantGoal_;

    void doneCb(const actionlib::SimpleClientGoalState& state,
           const sbc15_msgs::MoveManipulatorResultConstPtr& result);
};

#endif // MOVEITMOTION_H

