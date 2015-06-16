/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../states/global_state.h"
#include "../states/orient_to_sign.h"
#include "../states/drive_to_wall.h"
#include "../states/wait.h"
#include "../states/back_up.h"
#include "../states/initial_exploration.h"
#include "../states/find_delivery.h"
#include "../states/position_to_target.h"
#include "../states/go_to_delivery.h"
#include "../states/signal.h"
#include "../states/deliver_cube.h"

/// SYSTEM
#include <ros/ros.h>


class Next : public State
{
public:
    TriggeredEvent event_last_number_reached;
    TriggeredEvent event_next;

public:
    Next()
        : State(NULL),
          event_last_number_reached(this, "Done"),
          event_next(this, "Next sign")
    {
    }

    void entryAction()
    {
        next();
    }

    void iteration()
    {
    }

    void next()
    {
        GlobalState& global = GlobalState::getInstance();

        if(global.current_number >= 9) {
            event_last_number_reached.trigger();

        } else {
            ++global.current_number;
            event_next.trigger();
        }
    }
};

void restart()
{
    GlobalState::getInstance().current_number = 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sick14_sick10_test");
    ros::NodeHandle nh;

    sick14_fsm_global::waitForRosTime();

    restart();

    // STATES
    Initial init(State::NO_PARENT);
    InitialExploration explore_center(State::NO_PARENT, 2.0, 1.0);
    Next next;
    Wait error(State::NO_PARENT, 2, true);
    Error goal(State::NO_PARENT);
    FindDelivery find_delivery(State::NO_PARENT);
    OrientToSign orient_to_sign(State::NO_PARENT);
    DriveToWall drive_to_wall(State::NO_PARENT, 0.1, 10.0);
    Wait wait_at_wall(State::NO_PARENT, 2.0, true);
    BackUp drive_back_from_wall(State::NO_PARENT, .5, 0.5);
    GoToDelivery goto_delivery(State::NO_PARENT);

    // ACTIONS
    init.action_entry << boost::bind(&sick14_fsm_global::action::say, "Starting SICK robot day 2010 emulation mode!");
    init.action_entry << boost::bind(&restart);

    // TRANSITIONS
    init.event_default >> explore_center;
    explore_center.event_done >> goto_delivery;
    find_delivery.event_found_delivery >> goto_delivery;
//    goto_delivery.event_at_delivery >> orient_to_sign;
    goto_delivery.event_at_delivery >> drive_to_wall;
    //orient_to_sign.event_positioned >> drive_to_wall;
    //orient_to_sign.event_sign_not_found >> find_delivery;
    drive_to_wall.event_positioned >> wait_at_wall;
    wait_at_wall.event_done >> drive_back_from_wall;
    drive_back_from_wall.event_positioned >> next;
    goto_delivery.event_delivery_unknown >> find_delivery;
    next.event_last_number_reached >> goal;
    next.event_next >> find_delivery;
    error.event_done >> goal;

    StateMachine state_machine(&init);

    state_machine.run(boost::bind(&GlobalState::update, &GlobalState::getInstance()));

    return 0;
}

