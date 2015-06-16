/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../states/global_state.h"
#include "../states/wait.h"
#include "../states/wait_for_go_signal.h"
#include "../states/initial_exploration.h"
#include "../states/go_to_collection.h"
#include "../states/find_collection.h"
#include "../states/find_delivery.h"
#include "../states/get_cube.h"
#include "../states/position_to_target.h"
#include "../states/signal.h"
#include "../states/deliver_cube.h"

/// SYSTEM
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sick14_one_iteration_test");
    ros::NodeHandle nh;

    sick14_fsm_global::waitForRosTime();

    // STATES

    WaitForGoSignal init(State::NO_PARENT);
    Wait error(State::NO_PARENT, 2, true);
    Error goal(State::NO_PARENT);
    GetCube get_cube(State::NO_PARENT);
    DeliverCube deliver_cube(State::NO_PARENT);
    FindDelivery find_delivery(State::NO_PARENT);

    // ACTIONS
    init.action_entry << boost::bind(&sick14_fsm_global::action::say, "Testing one iteration");
    get_cube.signal_on.action_entry << boost::bind(&sick14_fsm_global::action::say, "Please give me a cube!");
    deliver_cube.signal_on.action_entry << boost::bind(&sick14_fsm_global::action::say, "Please take this cube!");
    deliver_cube.signal_off.action_entry << boost::bind(&sick14_fsm_global::action::say, "Thank you!");
    get_cube.eval_cube.event_no_cube << boost::bind(&sick14_fsm_global::action::say, "Did not get a cube!");
    deliver_cube.go_to_delivery.event_delivery_unknown << boost::bind(&sick14_fsm_global::action::say, "Delivery is unknown");
    goal.action_entry << boost::bind(&sick14_fsm_global::action::say, "success");
    error.action_entry << boost::bind(&sick14_fsm_global::action::say, "failure");

    // TRANSITIONS
    init.event_done >> get_cube;
    get_cube.go_to_collection.event_collection_unknown >> error;
    get_cube.back_up.event_positioned >> deliver_cube;
    deliver_cube.go_to_delivery.event_delivery_unknown >> find_delivery;
    deliver_cube.back_up.event_positioned >> goal;
    find_delivery.event_found_delivery >> deliver_cube;

    goal.event_default >> get_cube;

    StateMachine state_machine(&init);

    state_machine.run(boost::bind(&GlobalState::update, &GlobalState::getInstance()));

    return 0;
}

