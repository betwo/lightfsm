#ifndef PICKUPOBJECT_H
#define PICKUPOBJECT_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "follow_path.h"
#include "plan_arm_motion.h"
#include "visual_servoing.h"
#include "store_object.h"
#include "back_up.h"
#include "gripper_state.h"
#include "place_object.h"

/// PROJECT
#include <sbc15_msgs/Object.h>

class PickupObject : public MetaState
{
public:
    // states:
    PlanArmMotion plan_arm_motion;
    VisualServoing visual_servoing;

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

public:
    PickupObject(State* parent, bool store);

protected:
    double desiredFrequency() const;

private:
    bool store_;
};

#endif // PICKUPOBJECT_H
