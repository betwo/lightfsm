#ifndef STORE_OBJECT_H
#define STORE_OBJECT_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
#include "ros/ros.h"
class StoreObject: public State
{
public:
    TriggeredEvent object_stored;
    TriggeredEvent event_failure;

public:
    StoreObject(State* parent, int retries);

    void entryAction();
    void iteration();

private:
    ros::ServiceClient planedTrajectoryClient_;
    int retries_;
    int retries_left_;
    bool started_;
};

#endif // STORE_OBJECT_H
