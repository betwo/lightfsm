#ifndef VISUAL_SERVOING_H
#define VISUAL_SERVOING_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

#include <actionlib/client/simple_action_client.h>
#include <sbc15_msgs/visual_servoingAction.h>

class VisualServoing : public State
{
public:
    TriggeredEvent event_done;
    TriggeredEvent event_timeout;
    TriggeredEvent event_failure;
    TriggeredEvent event_out_of_range;
    TriggeredEvent event_servo_control_failed;
    TriggeredEvent event_no_object;

public:
    VisualServoing(State* parent, int retries);

    void entryAction();
    void iteration();

private:
    int retries_;
    int retries_left_;
    bool started_;

    sbc15_msgs::visual_servoingGoal goal_;
    actionlib::SimpleActionClient<sbc15_msgs::visual_servoingAction> client_;

    void doneCb(const actionlib::SimpleClientGoalState& state, const sbc15_msgs::visual_servoingResultConstPtr& result);
};

#endif  // VISUAL_SERVOING_H
