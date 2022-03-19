#ifndef EVENT_H
#define EVENT_H

/// COMPONENT
#include "lightfsm/transition.h"

/// SYSTEM
#include <vector>

class State;

class Event
{
public:
    Event(State* parent, const std::string& description);
    Event(const Event&) = delete;
    Event(Event&&) = delete;
    virtual ~Event();

    void connect(State* state, Guard guard = Guard(), Action action = Action());
    void connect(State* state, Action action);

    void connect(Event* event);

    void connect(Action action);

    virtual void getPossibleTransitions(std::vector<const Transition*>& possible_transitions) const;
    void getAllTransitions(std::vector<const Transition*>& transitions) const;

    std::string getDescription() const;

    void performTransitionActions();

    virtual void forwardEvent();

protected:
    State* parent_;
    std::string description_;

    std::vector<Transition> transitions_;
    std::vector<Event*> connected_events_;
    std::vector<Action> actions_on_transition_;
};

inline void operator>>(Event& e, Event& f)
{
    e.connect(&f);
}
inline void operator<<(Event& e, const Action& a)
{
    e.connect(a);
}
inline void operator<<(Event& e, const std::function<void()>& a)
{
    e.connect(Action(a));
}

#endif  // EVENT_H
