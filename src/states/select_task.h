#ifndef SELECTTASK_H
#define SELECTTASK_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

class SelectTask : public State
{
public:
    TriggeredEvent event_object_selected;
    TriggeredEvent event_all_objects_collected;
    TriggeredEvent event_object_unknown;

    void entryAction();
    void iteration();

public:
    SelectTask(State* parent);
};

#endif // SELECTTASK_H
