#ifndef FETCHOBJECT_H
#define FETCHOBJECT_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "goto_object.h"
#include "pickup_object.h"
#include "approach_object.h"
#include "wait.h"

/// SYSTEM
#include <tf/tf.h>

class FetchObject : public MetaState
{
public:
    TriggeredEvent event_object_unknown;
    TriggeredEvent event_object_fetched;
    TriggeredEvent event_failure;

public:
    FetchObject(State* parent, bool store);

public:
    GoToObject goto_object;
    ApproachObject approach;
    PickupObject pickup_object;

};

#endif // FETCHOBJECT_H
