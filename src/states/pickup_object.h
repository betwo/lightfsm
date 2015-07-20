#ifndef PICKUPOBJECT_H
#define PICKUPOBJECT_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "follow_path.h"

/// PROJECT
#include <sbc15_msgs/Object.h>

class PickupObject : public MetaState
{
public:
    TriggeredEvent event_object_pickedup;
    TriggeredEvent event_object_failure;

public:
    PickupObject(State* parent);

    void entryAction();
    void iteration();

protected:
    double desiredFrequency() const;
};

#endif // PICKUPOBJECT_H
