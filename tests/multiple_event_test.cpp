#include "lightfsm/action.h"
#include "lightfsm/state.h"
#include "lightfsm/state_machine.h"

#include "gtest/gtest.h"

class MultiEvent : public State
{
public:
    Event event_non_default;

    MultiEvent(State* parent) : State(parent), event_non_default(this, "another event")
    {
    }
};

class Sink : public State
{
public:
    Sink(State* parent) : State(parent)
    {
    }
};

class MultipleEventTests : public ::testing::Test
{
protected:
    MultipleEventTests()
    {
    }

    virtual ~MultipleEventTests()
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp()
    {
        // Code here will be called immediately after the constructor (right
        // before each test).
        value = 23;
    }

    virtual void TearDown()
    {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }
    int value;
};

TEST_F(MultipleEventTests, NoTransitionIsTakenWhenDefaultNoneConnected)
{
    MultiEvent fork(State::NO_PARENT);
    Sink sink_default(State::NO_PARENT);
    Sink sink_non_default(State::NO_PARENT);

    StateMachine state_machine(&fork);

    state_machine.step();
    ASSERT_EQ(state_machine.getState(), &fork);
    state_machine.step();
    ASSERT_EQ(state_machine.getState(), &fork);
}

TEST_F(MultipleEventTests, FirstTransitionIsTakenWhenMultiplePossible)
{
    MultiEvent fork(State::NO_PARENT);
    Sink sink_default(State::NO_PARENT);
    Sink sink_non_default(State::NO_PARENT);

    fork.event_default >> sink_default;
    fork.event_non_default >> sink_non_default;

    StateMachine state_machine(&fork);

    state_machine.step();
    ASSERT_EQ(state_machine.getState(), &fork);
    state_machine.step();
    ASSERT_EQ(state_machine.getState(), &sink_default);
}

TEST_F(MultipleEventTests, SecondTransitionIsTakenWhenDefaultNotConnected)
{
    MultiEvent fork(State::NO_PARENT);
    Sink sink_default(State::NO_PARENT);
    Sink sink_non_default(State::NO_PARENT);

    fork.event_non_default >> sink_non_default;

    StateMachine state_machine(&fork);

    state_machine.step();
    ASSERT_EQ(state_machine.getState(), &fork);
    state_machine.step();
    ASSERT_EQ(state_machine.getState(), &sink_non_default);
}
