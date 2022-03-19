#ifndef PREPLANNEDSTATE_H
#define PREPLANNEDSTATE_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
#include <sbc15_msgs/PreplannedTrajectories.h>
#include "ros/ros.h"

class PreplannedState : public State
{
public:
    TriggeredEvent event_done;
    TriggeredEvent event_failure;

public:
    PreplannedState(State* parent, int type, int retries);

    void entryAction();
    void iteration();

private:
    ros::ServiceClient planedTrajectoryClient_;

    int type_;
    int retries_;
    int retries_left_;
    bool started_;
};

#endif  // PREPLANNEDSTATE_H
