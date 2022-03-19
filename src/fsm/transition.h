#ifndef TRANSITION_H
#define TRANSITION_H

/// COMPONENT
#include "guard.h"
#include "action.h"

class State;
class Event;

class Transition
{
public:
    Transition(Event* trigger, State* follow_up, Guard guard, Action action);

    bool isPossible() const;
    Event* getEvent() const;
    State* getTarget() const;
    void performAction() const;

private:
    Event* trigger_;
    State* follow_up_;
    Guard guard_;
    Action action_;
};

#endif  // TRANSITION_H
