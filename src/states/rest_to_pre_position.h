#ifndef REST_TO_PRE_POSITION_H
#define REST_TO_PRE_POSITION_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "../states/preplanned_state.h"
#include "../states/gripper_state.h"
#include "../states/moveit_motion.h"
#include "ros/ros.h"

class RestToPrePosition: public MetaState //TODO use new plans
{
public:
    TriggeredEvent event_done;
    TriggeredEvent event_failure;

    GripperState semi_open_gripper;
    PreplannedState start_arm;
    GripperState open_gripper;
    //MoveitMotion pre_pos;
    PreplannedState pre_pos; //TODO use pre position: 0.106 0 0.27 1.5708 0

public:
    RestToPrePosition(State* parent, int retries);
};

#endif // REST_TO_PRE_POSITION_H
