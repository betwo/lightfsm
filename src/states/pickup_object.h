#ifndef PICKUPOBJECT_H
#define PICKUPOBJECT_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "follow_path.h"
#include "plan_arm_motion.h"
#include "visual_servoing.h"
#include "store_object.h"

/// PROJECT
#include <sbc15_msgs/Object.h>

class PickupObject : public MetaState
{
public:
    // states:
    PlanArmMotion plan_arm_motion;
    VisualServoing visual_servoing;
    StoreObject store_object;

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
