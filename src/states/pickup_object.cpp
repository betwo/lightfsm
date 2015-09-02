/// HEADER
#include "pickup_object.h"

/// COMPONENT
#include "../states/global_state.h"


PickupObject::PickupObject(State* parent)
    : MetaState(parent),
      event_object_pickedup(this, "The object has been reached"),
      event_object_failure(this, "The object pose is not known"),
      event_object_out_of_range(this, "Object is out of range"),
      event_servo_control_failed(this,"Control of arm failed"),
      plan_arm_motion(this,1),
      visual_servoing(this,2),
      store_object(this,1)
{
    event_entry_meta >> plan_arm_motion;

    plan_arm_motion.event_at_goal >> visual_servoing;
    visual_servoing.event_object_gripped >> store_object;
    store_object.object_stored >> event_object_pickedup;

    plan_arm_motion.event_failure >> event_object_failure;

    visual_servoing.event_failure >> plan_arm_motion;
    visual_servoing.event_out_of_range >> event_object_out_of_range;
    visual_servoing.event_timeout >> visual_servoing;
    visual_servoing.event_servo_control_failed >> event_servo_control_failed;
    visual_servoing.event_no_object >> plan_arm_motion; //go to object?

    store_object.event_failure >> store_object;
}

void PickupObject::entryAction()
{

}

void PickupObject::iteration()
{
}

double PickupObject::desiredFrequency() const
{
    return 1.0;
}
