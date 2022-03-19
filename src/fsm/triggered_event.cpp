/// HEADER
#include "triggered_event.h"

/// PROJECT
#include "state.h"
#include <iostream>

TriggeredEvent::TriggeredEvent(State* parent, const std::string& description)
  : Event(parent, description), triggered_(false)
{
}

void TriggeredEvent::trigger()
{
    triggered_ = true;
    std::cout << parent_->getName() << " triggered \"" << description_ << '\"' << std::endl;
}

void TriggeredEvent::getPossibleTransitions(std::vector<const Transition*>& possible_transitions) const
{
    if (triggered_) {
        std::cout << parent_->getName() << ": " << description_ << " is triggered" << std::endl;
        Event::getPossibleTransitions(possible_transitions);

        triggered_ = false;
    }
}

void TriggeredEvent::forwardEvent()
{
    trigger();
}
