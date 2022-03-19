/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state_machine_basic_executor.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"
#include "../global.h"
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

#include <chrono>

class Entry : public State
{
public:
    TriggeredEvent event_done;

public:
    Entry(State* parent, std::chrono::seconds duration) : State(parent), event_done(this, "done"), duration_(duration)
    {
    }

protected:
    void entryAction()
    {
        auto now = std::chrono::steady_clock::now();
        continue_at_ = now + duration_;
    }

    void iteration()
    {
        std::cout << "Iteration " << getName() << '\n';
        auto now = std::chrono::steady_clock::now();
        if (now >= continue_at_) {
            event_done.trigger();
        }
    }

    double desiredFrequency() const
    {
        return 20.0;
    }

private:
    std::chrono::seconds duration_;
    std::chrono::_V2::steady_clock::time_point continue_at_;
};

class Exit : public State
{
public:
    Exit(State* parent) : State(parent)
    {
    }

    bool isTerminal() const override
    {
        return true;
    }
};

void tick(State* current_state)
{
    std::cout << "Ticking state " << current_state->getName() << '\n';
}

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <time-to-sleep>" << std::endl;
        return 1;
    }
    // STATES
    Entry entry(State::NO_PARENT, std::chrono::seconds(std::atoi(argv[1])));
    Exit exit(State::NO_PARENT);

    // ACTIONS
    entry.action_entry << []() { std::cout << "Entry: action entry." << '\n'; };
    entry.event_done << []() { std::cout << "Entry: event_done." << '\n'; };
    entry.action_exit << []() { std::cout << "Entry: action exit." << '\n'; };

    entry.event_done >> exit;
    exit.action_entry << []() { std::cout << "Exit: action entry." << '\n'; };
    exit.action_exit << []() { std::cout << "Exit: action exit." << '\n'; };

    StateMachine state_machine(&entry);

    StateMachineBasicExecutor executor(state_machine);
    executor.run([](State* state) { tick(state); });

    return 0;
}
