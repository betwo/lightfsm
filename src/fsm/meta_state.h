#ifndef META_STATE_H
#define META_STATE_H

/// COMPONENT
#include "state.h"
#include "triggered_event.h"

class MetaState : public State
{
protected:
    TriggeredEvent event_entry_meta;

public:
    TriggeredEvent event_exit_meta;

public:
    MetaState(State *parent);

    void entryAction();
};

#endif // META_STATE_H
