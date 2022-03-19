#ifndef SELECTTASK_H
#define SELECTTASK_H

/// COMPONENT
#include "lightfsm/state.h"
#include "lightfsm/triggered_event.h"

class SelectTask : public State
{
public:
    TriggeredEvent event_object_selected;
    TriggeredEvent event_all_objects_collected;
    TriggeredEvent event_object_unknown;
    TriggeredEvent event_first_move;

    void entryAction();
    void iteration();

public:
    SelectTask(State* parent);

private:
    bool first_;
};

#endif  // SELECTTASK_H
