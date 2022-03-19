#ifndef POSITION_TO_BASE_H
#define POSITION_TO_BASE_H
/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
#include "ros/ros.h"

class PositionToBase : public State

{
public:
    TriggeredEvent event_done;
    TriggeredEvent event_failure;

public:
    PositionToBase(State* parent);

    void entryAction();
    void iteration();
};

#endif  // POSITION_TO_BASE_H
