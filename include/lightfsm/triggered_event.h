#ifndef TRIGGERED_EVENT_H
#define TRIGGERED_EVENT_H

/// COMPONENT
#include "lightfsm/event.h"

class TriggeredEvent : public Event
{
public:
    TriggeredEvent(State* parent, const std::string& description);

    void trigger();

    virtual void getPossibleTransitions(std::vector<const Transition*>& possible_transitions) const;

protected:
    virtual void forwardEvent();

private:
    mutable bool triggered_;
};

#endif  // TRIGGERED_EVENT_H
