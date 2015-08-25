#ifndef STORE_OBJECT_H
#define STORE_OBJECT_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

class StoreObject: public State
{
public:
    TriggeredEvent object_stored;
    TriggeredEvent event_failure;

public:
    StoreObject(State* parent);
};

#endif // STORE_OBJECT_H
