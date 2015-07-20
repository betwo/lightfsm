#ifndef FETCHOBJECT_H
#define FETCHOBJECT_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "goto_object.h"
#include "pickup_object.h"

/// SYSTEM
#include <tf/tf.h>

class FetchObject : public MetaState
{
public:
    TriggeredEvent event_object_unknown;

public:
    FetchObject(State* parent);

public:
    GoToObject goto_object;
    PickupObject pickup_object;

};

#endif // FETCHOBJECT_H
