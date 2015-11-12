#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/// COMPONENT
#include "state.h"

class StateMachine
{
public:
    StateMachine(State* initial_state);

    void run(boost::function<void(State *)> callback);
    bool step();

    std::string generateGraphDescription() const;

    void reset();
    void gotoState(State* state);

private:
    void check();
    void perform(const Transition& perform);

private:

    template <class Stream>
    Stream& printConnections(Stream& stream, const State* s, const std::string& prefix) const;

    template <class Stream>
    Stream& printState(Stream& stream, const State* s, const std::string& prefix) const;

private:
    State* start_state_;
    State* state_;

    bool reset_;
    State* reset_state_;
};

#endif // STATE_MACHINE_H
