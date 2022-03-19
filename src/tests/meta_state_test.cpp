#include "lightfsm/event.h"
#include "lightfsm/meta_state.h"
#include "lightfsm/state_machine.h"
#include "../global.h"

#include "gtest/gtest.h"

#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>

class MetaStateTest : public ::testing::Test
{
protected:
    MetaStateTest()
    {
        ros::Time::init();
    }

    virtual ~MetaStateTest()
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp()
    {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown()
    {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }
};

class MockupMetaState : public MetaState
{
public:
    MockupMetaState()
      : MetaState(State::NO_PARENT)
      ,

      init(this)
      , intermediate(this)
      , goal(this)

    {
        event_entry_meta >> init;

        init.event_default >> intermediate;
        intermediate.event_default >> goal;

        //        goal.event_default >> event_exit_meta;
        goal.event_default.connect(&event_exit_meta);
    }

public:
    Initial init;
    Initial intermediate;
    Quit goal;
};

namespace
{
void set(bool* dst)
{
    *dst = true;
}
}  // namespace

TEST_F(MetaStateTest, EventConnectionIsFollowed)
{
    bool meta_entry_called = false;
    bool meta_init_called = false;
    bool meta_intermediate_called = false;
    bool meta_goal_called = false;
    bool meta_exit_called = false;
    bool goal_called = false;

    Initial init(State::NO_PARENT);
    MockupMetaState meta;
    Quit goal(State::NO_PARENT);
    init.event_default >> meta;
    //    meta.goal.event_default >> goal;
    meta.event_exit_meta >> goal;

    meta.action_entry << Action([&]() { set(&meta_entry_called); });
    meta.init.action_entry << Action([&]() { set(&meta_init_called); });
    meta.intermediate.action_entry << Action([&]() { set(&meta_intermediate_called); });
    meta.goal.action_entry << Action([&]() { set(&meta_goal_called); });
    meta.action_exit << Action([&]() { set(&meta_exit_called); });

    goal.action_entry << Action([&]() { set(&goal_called); });

    StateMachine state_machine(&init);

    state_machine.step();  // init -> meta
    ASSERT_TRUE(meta_entry_called);

    state_machine.step();  // meta -> meta.init
    ASSERT_TRUE(meta_init_called);

    state_machine.step();  // meta.init -> meta.intermediate
    ASSERT_TRUE(meta_intermediate_called);

    state_machine.step();  // meta.intermediate -> meta.goal
    ASSERT_TRUE(meta_goal_called);

    state_machine.step();  // meta.goal -> goal
    ASSERT_TRUE(meta_exit_called);
    ASSERT_TRUE(goal_called);
}

class MockupMetaStateWithEventForwarding : public MetaState
{
public:
    TriggeredEvent event_done;

    MockupMetaStateWithEventForwarding()
      : MetaState(State::NO_PARENT)
      ,

      event_done(this, "done was called!")
      ,

      init(State::NO_PARENT)
      , intermediate(State::NO_PARENT)
      , goal(State::NO_PARENT)
    {
        event_entry_meta >> init;

        init.event_default >> intermediate;
        intermediate.event_default >> event_done;
    }

public:
    Initial init;
    Initial intermediate;
    Initial goal;
};

TEST_F(MetaStateTest, EventForwardingIsFollowedFromSubState)
{
    bool goal_called = false;

    Initial init(State::NO_PARENT);
    MockupMetaStateWithEventForwarding meta;
    Quit goal(State::NO_PARENT);
    init.event_default >> meta;
    meta.event_done >> goal;

    goal.action_entry << Action(std::bind(&set, &goal_called));

    StateMachine state_machine(&init);

    state_machine.step();  // init -> meta
    state_machine.step();  // meta -> meta.init
    state_machine.step();  // meta.init -> meta.intermediate
    state_machine.step();  // meta.intermediate -> goal
    ASSERT_TRUE(goal_called);
}
