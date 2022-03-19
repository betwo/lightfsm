#ifndef POSITION_TO_BASE_H
#define POSITION_TO_BASE_H
/// COMPONENT
#include "lightfsm/state.h"
#include "lightfsm/triggered_event.h"
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
