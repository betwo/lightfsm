#ifndef GRIPPERSTATE_H
#define GRIPPERSTATE_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
#include <sbc15_msgs/GripperServices.h>
#include "ros/ros.h"

class GripperState : public State
{
public:
    TriggeredEvent event_done;

public:
    GripperState(State* parent, int type, double effort);

    void entryAction();
    void iteration();

private:
    ros::ServiceClient gripper_client_;

    int type_;
    double cup_effort_;
    double bat_effort_;
    double effort_;

};
#endif // GRIPPERSTATE_H
