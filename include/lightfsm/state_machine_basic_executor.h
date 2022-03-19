#ifndef STATE_MACHINE_BASIC_EXECUTOR_H
#define STATE_MACHINE_BASIC_EXECUTOR_H

/// COMPONENT
#include "lightfsm/state_machine_executor.h"

class StateMachineBasicExecutor : public StateMachineExecutor
{
public:
    StateMachineBasicExecutor(StateMachine& state_machine);

    void run(std::function<void(State*)> callback) override;
};

#endif  // STATE_MACHINE_BASIC_EXECUTOR_H