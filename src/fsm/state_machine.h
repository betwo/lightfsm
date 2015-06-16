#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/// COMPONENT
#include "state.h"

class StateMachine
{
public:
    StateMachine(State* initial_state);

    void run(boost::function<void()> callback);
    bool step();

private:
    void check();
    void perform(const Transition& perform);

private:
    State* state_;
};

#endif // STATE_MACHINE_H
