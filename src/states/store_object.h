#ifndef STORE_OBJECT_H
#define STORE_OBJECT_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "../states/preplanned_state.h"
#include "../states/gripper_state.h"
#include "ros/ros.h"

class StoreObject: public MetaState
{
public:
    TriggeredEvent object_stored;
    TriggeredEvent event_failure;

    TriggeredEvent event_cup;
    TriggeredEvent event_battery;


    PreplannedState place_cup;
    GripperState open_gripper_cup;
    PreplannedState rest_position_cup;

    PreplannedState place_battery;
    GripperState open_gripper_battery;
    PreplannedState rest_position_battery;

public:
    StoreObject(State* parent, int retries);

public:
    void entryAction();
};

#endif // STORE_OBJECT_H
