#ifndef PLACE_CUP_H
#define PLACE_CUP_H

#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "rest_to_pre_position.h"
#include "store_object.h"
#include "back_up.h"
#include "gripper_state.h"
#include "position_to_base.h"
#include "moveit_motion.h"
#include "recorded_trajectory.h"



/// PROJECT
#include <sbc15_msgs/Object.h>
class PlaceCup : public MetaState
{
public:
    // states:
    PositionToBase goToBase;
    RecordedTrajectory goToCup;
    GripperState closeGripper;
    RecordedTrajectory takeCup;
    MoveitMotion placeCup;
    GripperState openGri;
    MoveitMotion goToCrane;
    PreplannedState goToRest;


public:
    TriggeredEvent event_cup_placed;
    TriggeredEvent event_failure;

public:
    PlaceCup(State* parent);

//    void entryAction();


};

#endif // PLACE_CUP_H
