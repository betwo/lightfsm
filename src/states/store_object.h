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

    PreplannedState place_object;
    GripperState open_gripper;
    PreplannedState rest_position;

public:
    StoreObject(State* parent, int retries);
};

#endif // STORE_OBJECT_H
