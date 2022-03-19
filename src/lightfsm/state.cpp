/// HEADER
#include "lightfsm/state.h"

/// SYSTEM
#include <typeinfo>
#include <cxxabi.h>

State* State::NO_PARENT = NULL;

std::vector<State*> State::g_states;

State::State(State* parent) : event_default(this, "default"), parent_(parent), uuid_(nextId())
{
    g_states.push_back(this);

    if (parent != NO_PARENT) {
        parent->registerChildState(this);
    }
}

State::~State()
{
    auto pos = std::find(g_states.begin(), g_states.end(), this);
    g_states.erase(pos);
}

void State::registerChildState(State* /*child*/)
{
    // do nothing
}

int State::getUniqueId() const
{
    return uuid_;
}

int State::nextId()
{
    static int id = 0;
    return id++;
}

std::string State::getName() const
{
    int status;
    std::string full_name(abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status));

    return full_name;
}

bool State::isTerminal() const
{
    return false;
}

void State::performEntryAction()
{
    for (std::vector<Action>::const_iterator a = action_entry.begin(); a != action_entry.end(); ++a) {
        const Action& action = *a;
        action.perform();
    }

    entryAction();
}

void State::performExitAction()
{
    exitAction();

    for (std::vector<Action>::const_iterator a = action_exit.begin(); a != action_exit.end(); ++a) {
        const Action& action = *a;
        action.perform();
    }
}

void State::entryAction()
{
}

void State::exitAction()
{
}

void State::iteration()
{
}

double State::desiredFrequency() const
{
    return 1.0;
}

void State::tick(std::vector<const Transition*>& possible_transitions)
{
    // do work
    iteration();

    // check events
    for (std::vector<Event*>::const_iterator e = events_.begin(); e != events_.end(); ++e) {
        const Event* event = *e;

        event->getPossibleTransitions(possible_transitions);
    }
}

void State::registerEvent(Event* event)
{
    events_.push_back(event);
}

State* State::getParent() const
{
    return parent_;
}

std::vector<Event*> State::getEvents() const
{
    return events_;
}
