/// HEADER
#include "event.h"

/// COMPONENT
#include "state.h"

/// SYSTEM
#include <iostream>

Event::Event(State* parent, const std::string& description) : parent_(parent), description_(description)
{
    parent_->registerEvent(this);
}

Event::~Event()
{
}

std::string Event::getDescription() const
{
    return description_;
}

void Event::connect(State* state, Guard guard, Action action)
{
    transitions_.push_back(Transition(this, state, guard, action));
}
void Event::connect(State* state, Action action)
{
    transitions_.push_back(Transition(this, state, Guard(), action));
}

void Event::connect(Event* event)
{
    connected_events_.push_back(event);
}

void Event::connect(Action action)
{
    actions_on_transition_.push_back(action);
}

void Event::getPossibleTransitions(std::vector<const Transition*>& possible_transitions) const
{
    for (std::vector<Transition>::const_iterator t = transitions_.begin(); t != transitions_.end(); ++t) {
        const Transition& transition = *t;

        if (transition.isPossible()) {
            possible_transitions.push_back(&transition);
        }
    }
    for (std::vector<Event*>::const_iterator e = connected_events_.begin(); e != connected_events_.end(); ++e) {
        Event* event = *e;

        event->forwardEvent();
        event->getPossibleTransitions(possible_transitions);
    }
}

void Event::getAllTransitions(std::vector<const Transition*>& transitions) const
{
    for (std::vector<Transition>::const_iterator t = transitions_.begin(); t != transitions_.end(); ++t) {
        const Transition& transition = *t;

        transitions.push_back(&transition);
    }
    for (std::vector<Event*>::const_iterator e = connected_events_.begin(); e != connected_events_.end(); ++e) {
        Event* event = *e;

        event->getAllTransitions(transitions);
    }
}

void Event::performTransitionActions()
{
    for (std::vector<Action>::iterator a = actions_on_transition_.begin(); a != actions_on_transition_.end(); ++a) {
        a->perform();
    }
}

void Event::forwardEvent()
{
}
