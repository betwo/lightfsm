/// HEADER
#include "transition.h"

/// COMPONENT
#include "event.h"

Transition::Transition(Event* trigger, State* follow_up, Guard guard, Action action)
    : trigger_(trigger), follow_up_(follow_up), guard_(guard), action_(action)
{
}

bool Transition::isPossible() const
{
    return guard_.evaluate();
}

Event* Transition::getEvent() const
{
    return trigger_;
}

State* Transition::getTarget() const
{
    return follow_up_;
}

void Transition::performAction() const
{
    action_.perform();
}
