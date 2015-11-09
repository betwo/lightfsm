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
#include "preplanned_state.h"
#include "visual_servoing.h"


/// PROJECT
#include <sbc15_msgs/Object.h>
class PlaceCup : public MetaState
{

public:
public:
    // states:
    PositionToBase goToBase;
    PreplannedState prePos;      // TODO: Replace with take cup from storage
    MoveitMotion preVs;          // TODO: remove
    VisualServoing visualServo;  // TODO: remove
    MoveitMotion planToCrane;   // TODO: remove
    MoveitMotion planToCrane2;   // TODO: remove
    MoveitMotion planToCrane3;   // TODO: remove
    MoveitMotion placeCup;
    GripperState openGri;
    GripperState openGri2;
    PreplannedState goToRest;


public:
    TriggeredEvent event_cup_placed;
    TriggeredEvent event_failure;

public:
    PlaceCup(State* parent);


};

#endif // PLACE_CUP_H
