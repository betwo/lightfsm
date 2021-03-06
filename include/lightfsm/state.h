#ifndef STATE_H
#define STATE_H

/// COMPONENT
#include "lightfsm/action.h"
#include "lightfsm/actions.h"
#include "lightfsm/event.h"

/// SYSTEM
#include <vector>

class Event;
class Transition;

class State
{
public:
    static State* NO_PARENT;

private:
    std::vector<Event*> events_;

public:
    Event event_default;

    static std::vector<State*> g_states;

public:
    Actions action_entry;
    Actions action_exit;

public:
    State(State* parent);
    virtual ~State();

    void tick(std::vector<const Transition*>& possible_transitions);

    virtual bool isTerminal() const;
    virtual double desiredFrequency() const;

    void performEntryAction();
    void performExitAction();

    State* getParent() const;
    std::string getName() const;

    std::vector<Event*> getEvents() const;

    int getUniqueId() const;

protected:
    virtual void entryAction();
    virtual void exitAction();
    virtual void iteration();

    virtual void registerChildState(State* child);

private:
    friend class Event;
    void registerEvent(Event* event);

    static int nextId();

private:
    State* parent_;

    int uuid_;
};

inline void operator>>(Event& e, State& s)
{
    e.connect(&s);
}
inline void operator>>(Event* e, State& s)
{
    e->connect(&s);
}

inline void operator>>(Event& e, State* s)
{
    e.connect(s);
}

#endif  // STATE_H
