#ifndef STATE_MACHINE_EXECUTOR_H
#define STATE_MACHINE_EXECUTOR_H

/// COMPONENT
#include "state_machine.h"

class StateMachineExecutor
{
public:
    StateMachineExecutor(StateMachine& state_machine);

    virtual void run(std::function<void(State*)> callback) = 0;

protected:
    StateMachine& state_machine_;
};

#endif  // STATE_MACHINE_H
