/// HEADER
#include "pickup_object.h"

/// COMPONENT
#include "../states/global_state.h"


PickupObject::PickupObject(State* parent)
    : MetaState(parent),
      event_object_pickedup(this, "The object has been reached"),
      event_object_failure(this, "The object pose is not known")
{

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
