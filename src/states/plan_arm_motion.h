#ifndef PLAN_ARM_MOTION_H
#define PLAN_ARM_MOTION_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "../states/preplanned_state.h"
#include "../states/gripper_state.h"
#include "../states/moveit_motion.h"
#include "ros/ros.h"

class PlanArmMotion: public MetaState
{
public:
    TriggeredEvent event_done;
    TriggeredEvent event_failure;

    GripperState semi_open_gripper;
    PreplannedState start_arm;
    GripperState open_gripper;
    //MoveitMotion pre_pos;
    PreplannedState pre_pos;

public:
    PlanArmMotion(State* parent, int retries);
};

#endif // PLAN_ARM_MOTION_H
