#ifndef STORE_OBJECT_H
#define STORE_OBJECT_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "../states/preplanned_state.h"
#include "../states/gripper_state.h"
#include "../states/recorded_trajectory.h"
#include "ros/ros.h"

class StoreObject: public MetaState
{
public:
    TriggeredEvent object_stored;
    TriggeredEvent event_failure;

    TriggeredEvent event_cup;
    TriggeredEvent event_battery;


    RecordedTrajectory place_cup1;
    GripperState open_gripper_cup;
    RecordedTrajectory place_cup2;
    RecordedTrajectory rest_position_cup;

    RecordedTrajectory place_battery1;
    GripperState open_gripper_battery;
    RecordedTrajectory place_battery2;
    RecordedTrajectory rest_position_battery;
    RecordedTrajectory sleep_from_battery;
    GripperState gripper_semi_close;

public:
    StoreObject(State* parent, int retries);

public:
    void entryAction();
};

#endif // STORE_OBJECT_H
