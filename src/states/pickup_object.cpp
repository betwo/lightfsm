/// HEADER
#include "pickup_object.h"

/// COMPONENT
#include "../states/global_state.h"


PickupObject::PickupObject(State* parent)
    : MetaState(parent),
      event_object_pickedup(this, "The object has been reached"),
      event_object_failure(this, "The object pose is not known"),
      plan_arm_motion(this),
      visual_servoing(this),
      store_object(this)
{
    event_entry_meta >> plan_arm_motion;

    plan_arm_motion.event_at_goal >> visual_servoing;
    visual_servoing.event_object_gripped >> store_object;
    store_object.object_stored >> event_object_pickedup;

    plan_arm_motion.event_failure >> plan_arm_motion;
    visual_servoing.event_failure >> plan_arm_motion;
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
