/// HEADER
#include "lightfsm/state_machine_basic_executor.h"

/// COMPONENT
#include "lightfsm/transition.h"
#include "lightfsm/meta_state.h"

/// SYSTEM
#include <signal.h>
#include <chrono>
#include <iostream>
#include <thread>

StateMachineBasicExecutor::StateMachineBasicExecutor(StateMachine& state_machine) : StateMachineExecutor(state_machine)
{
}

namespace
{
bool keep_running = true;
void signalhandler(int /* signal */)
{
    keep_running = false;
}
}  // namespace

void StateMachineBasicExecutor::run(std::function<void(State*)> callback)
{
    State* initial_state = state_machine_.getState();
    std::cout << "starting basic executor with state " << initial_state->getName() << '\n';
    initial_state->performEntryAction();

    signal(SIGINT, signalhandler);

    while (keep_running) {
        callback(state_machine_.getState());

        state_machine_.step();

        if (state_machine_.getState()->isTerminal()) {
            state_machine_.shutdown();
            return;
        }

        const double sleep_in_seconds = 1.0 / state_machine_.getState()->desiredFrequency();
        const std::uint64_t sleep_milliseconds(sleep_in_seconds * 1000.);
        std::chrono::milliseconds sleep_time(sleep_milliseconds);
        std::this_thread::sleep_for(sleep_time);
    }
}
