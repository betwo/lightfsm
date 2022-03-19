#ifndef STATE_MACHINE_ROS_EXECUTOR_H
#define STATE_MACHINE_ROS_EXECUTOR_H

/// COMPONENT
#include "lightfsm/state_machine_executor.h"

class StateMachineRosExecutor : public StateMachineExecutor
{
public:
    StateMachineRosExecutor(StateMachine& state_machine);

    void run(std::function<void(State*)> callback) override;
};

#endif  // STATE_MACHINE_ROS_EXECUTOR_H