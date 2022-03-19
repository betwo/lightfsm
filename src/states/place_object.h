#ifndef PLACE_OBJECT_H
#define PLACE_OBJECT_H

/// COMPONENT
#include "lightfsm/meta_state.h"
#include "lightfsm/triggered_event.h"
#include "../states/preplanned_state.h"
#include "../states/gripper_state.h"
#include "ros/ros.h"

class PlaceObject : public MetaState
{
public:
    TriggeredEvent event_object_placed;

    PreplannedState place_object;
    GripperState open_gripper;
    PreplannedState pre_rest_position;
    GripperState close_gripper;
    PreplannedState rest_position;

public:
    PlaceObject(State* parent, int retries);
};

#endif  // PLACE_OBJECT_H
