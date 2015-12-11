#ifndef PICKUPOBJECT_H
#define PICKUPOBJECT_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "follow_path.h"
#include "rest_to_pre_position.h"
#include "visual_servoing.h"
#include "store_object.h"
#include "back_up.h"
#include "gripper_state.h"
#include "place_object.h"
#include "moveit_motion.h"

/// PROJECT
#include <sbc15_msgs/Object.h>

class PickupObject : public MetaState
{
public:
    // states:
    RestToPrePosition plan_arm_motion;
    VisualServoing visual_servoing;
    GripperState grab_obj;
    MoveitMotion pose_interim;

    StoreObject store_object;
    PlaceObject place_object;
    GripperState open_gripper;

    PreplannedState pre_pos;
    BackUp drive_forward;

    PreplannedState back_up;
    BackUp drive_backward;


    GripperState abort;
    PreplannedState abort2;

public:
    TriggeredEvent event_object_pickedup;
    TriggeredEvent event_object_failure;
    TriggeredEvent event_object_out_of_range;
    TriggeredEvent event_servo_control_failed;
    TriggeredEvent event_planning_failed;

public:
    PickupObject(State* parent, bool store);

    static ArmGoal createInterimPose();

protected:
    double desiredFrequency() const;

private:
    bool store_;
};

#endif // PICKUPOBJECT_H
