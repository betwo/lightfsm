#include "lightfsm/event.h"
#include "lightfsm/triggered_event.h"
#include "lightfsm/state.h"
#include "lightfsm/state_machine.h"

#include "gtest/gtest.h"

class Initial : public State
{
public:
    Initial(State* parent) : State(parent)
    {
    }
};

class Quit : public State
{
public:
    Quit(State* parent) : State(parent)
    {
    }
};

class EventTest : public ::testing::Test
{
protected:
    EventTest() : default_value(23)
    {
    }

    virtual ~EventTest()
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp()
    {
        // Code here will be called immediately after the constructor (right
        // before each test).
        value = default_value;
    }

    virtual void TearDown()
    {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }
    int value;
    const int default_value;
};

namespace
{
void set(int* dst, int val)
{
    *dst = val;
}
}  // namespace

TEST_F(EventTest, EventCanBeConnected)
{
    Initial init(State::NO_PARENT);
    Quit goal(State::NO_PARENT);

    Event e(&init, "description");

    e >> goal;

    init.action_entry << Action([&]() { set(&value, 42); });

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(42, value);
}

TEST_F(EventTest, TriggeredEventIsFollowedWhenTriggered)
{
    Initial init(State::NO_PARENT);
    Quit goal(State::NO_PARENT);

    TriggeredEvent e(&init, "test trigger");

    e >> goal;

    init.action_entry << Action([&]() { set(&value, 42); });

    StateMachine state_machine(&init);

    e.trigger();
    state_machine.step();

    ASSERT_EQ(42, value);
}

TEST_F(EventTest, TriggeredEventIsNotFollowedWhenNotTriggered)
{
    Initial init(State::NO_PARENT);
    Quit goal(State::NO_PARENT);

    TriggeredEvent e(&init, "test trigger");

    e >> goal;

    goal.action_entry << Action([&]() { set(&value, 42); });

    StateMachine state_machine(&init);

    // not calling e.trigger();
    state_machine.step();

    ASSERT_EQ(default_value, value);
}

TEST_F(EventTest, EventConnectionIsFollowed)
{
    Initial init(State::NO_PARENT);
    Quit goal(State::NO_PARENT);
    init.event_default >> goal;

    init.action_entry << Action([&]() { set(&value, 42); });

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(42, value);
}

TEST_F(EventTest, EventForwardingWorks)
{
    Initial init(State::NO_PARENT);
    Initial intermediate(State::NO_PARENT);
    Quit goal(State::NO_PARENT);
    init.event_default >> intermediate.event_default;
    intermediate.event_default >> goal;
    init.action_entry << Action([&]() { set(&value, 42); });

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(42, value);
}

TEST_F(EventTest, EventConnectionWithDefaultGuardIsFollowed)
{
    Initial init(State::NO_PARENT);
    Quit goal(State::NO_PARENT);

    Guard guard;
    init.event_default.connect(&goal, guard, Action([&]() { set(&value, 42); }));

    StateMachine state_machine(&init);

    state_machine.step();
    ASSERT_EQ(default_value, value);

    state_machine.step();
    ASSERT_EQ(42, value);
}

TEST_F(EventTest, EventConnectionWithoutGuardIsFollowed)
{
    Initial init(State::NO_PARENT);
    Quit goal(State::NO_PARENT);

    init.event_default.connect(&goal, Action([&]() { set(&value, 42); }));

    StateMachine state_machine(&init);

    state_machine.step();
    ASSERT_EQ(default_value, value);

    state_machine.step();
    ASSERT_EQ(42, value);
}

TEST_F(EventTest, EventConnectionWithUnsatisfiedGuardIsNotFollowed)
{
    Initial init(State::NO_PARENT);
    Quit goal(State::NO_PARENT);

    std::function<bool()> condition = []() { return false; };
    Guard guard(condition);
    init.event_default.connect(&goal, guard, Action([&]() { set(&value, 42); }));

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(default_value, value);
}

TEST_F(EventTest, EventConnectionWithSatisfiedGuardIsFollowed)
{
    Initial init(State::NO_PARENT);
    Quit goal(State::NO_PARENT);

    std::function<bool()> condition = []() { return true; };
    Guard guard(condition);
    init.event_default.connect(&goal, guard, Action([&]() { set(&value, 42); }));

    StateMachine state_machine(&init);

    state_machine.step();
    ASSERT_EQ(default_value, value);

    state_machine.step();
    ASSERT_EQ(42, value);
}
