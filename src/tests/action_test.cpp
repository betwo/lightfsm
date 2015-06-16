#include "../fsm/action.h"
#include "../fsm/state.h"
#include "../fsm/state_machine.h"
#include "../global.h"

#include "gtest/gtest.h"

#include <boost/bind.hpp>


class ActionTest : public ::testing::Test {
protected:
    ActionTest()
    {
        ros::Time::init();
    }

    virtual ~ActionTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() {
        // Code here will be called immediately after the constructor (right
        // before each test).
        value = 23;
    }

    virtual void TearDown() {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }
    int value;
};

namespace {
void set(int* dst, int val)
{
    *dst = val;
}
}


TEST_F(ActionTest, ActionIsPerformed) {
    Initial init(State::NO_PARENT);
    Action a(boost::bind(&set, &value, 42));
    init.action_entry << a;

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(42, value);
}

TEST_F(ActionTest, ActionCanBeDirectlyBoundPerformed) {
    Initial init(State::NO_PARENT);
    init.action_entry << boost::bind(&set, &value, 42);

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(42, value);
}


TEST_F(ActionTest, ActionCanBeCopied) {
    Initial init(State::NO_PARENT);
    Action a(boost::bind(&set, &value, 42));
    Action a_copy = a;
    init.action_entry << a_copy;

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(42, value);
}

TEST_F(ActionTest, ActionCanBeAssigned) {
    Initial init(State::NO_PARENT);
    Action a(boost::bind(&set, &value, 42));
    Action a_copy;
    a_copy = a;
    init.action_entry << a_copy;

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(42, value);
}

TEST_F(ActionTest, EmptyActionIsPossible) {
    Initial init(State::NO_PARENT);
    init.action_entry.push_back(Action());

    StateMachine state_machine(&init);

    state_machine.step();

    SUCCEED();
}

TEST_F(ActionTest, ActionCanBeAssignedToAnEvent) {
    Initial init(State::NO_PARENT);
    Initial goal(State::NO_PARENT);
    int goal_value = 23;
    init.event_default << boost::bind(&set, &value, 42);

    init.event_default >> goal;
    goal.action_entry << boost::bind(&set, &goal_value, 42);

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(42, goal_value);
    ASSERT_EQ(42, value);
}


TEST_F(ActionTest, ActionAssignedToAnEventIsOnlyExecutedWhenEventIsTriggered) {
    Initial init(State::NO_PARENT);
    init.event_default << boost::bind(&set, &value, 42);

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(23, value);
}


namespace {
void order(int* no, int expected, bool* ok)
{
    *ok &= (expected == *no);
    ++(*no);
}
}



TEST_F(ActionTest, ActionsAreCalledInCorrectOrder) {
    Initial init(State::NO_PARENT);
    Initial goal(State::NO_PARENT);

    int no = 0;
    bool ok = true;

    init.event_default.connect(&goal, Action(boost::bind(&order, &no, 2, &ok)));

    init.action_exit << boost::bind(&order, &no, 0, &ok);
    init.event_default << boost::bind(&order, &no, 1, &ok);
    goal.action_entry << boost::bind(&order, &no, 3, &ok);

    StateMachine state_machine(&init);

    state_machine.step();

    ASSERT_EQ(4, no);
    ASSERT_TRUE(ok);
}
