#ifndef VISUAL_SERVOING_H
#define VISUAL_SERVOING_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

class VisualServoing: public State
{
public:
    TriggeredEvent event_object_gripped;
    TriggeredEvent event_failure;
public:
    VisualServoing();

    void entryAction();
    void iteration();


};

#endif // VISUAL_SERVOING_H
