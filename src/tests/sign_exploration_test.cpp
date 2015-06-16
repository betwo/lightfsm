/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../states/find_delivery.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sick14_state_machine_node_sign_exploration_test");
    ros::NodeHandle nh("~");

    sick14_fsm_global::waitForRosTime();

    GlobalState& global = GlobalState::getInstance();

    nh.param("number", global.current_number, -1);

    ROS_INFO_STREAM("starting sign exploration with target sign " << global.current_number);

    // STATES
    Initial init(State::NO_PARENT);
    FindDelivery find_delivery(State::NO_PARENT);
    Quit exit(State::NO_PARENT);

    // ACTIONS
    init.action_entry.push_back(Action(boost::bind(&sick14_fsm_global::action::say, "Testing Sign Exploration")));
    exit.action_entry.push_back(Action(boost::bind(&sick14_fsm_global::action::say, "Found Sign")));

    // TRANSITIONS
    init.event_default >> find_delivery;
    find_delivery.event_found_delivery >> exit;

    StateMachine state_machine(&init);


    ROS_INFO("running state machine");
    state_machine.run(boost::bind(&GlobalState::update, &GlobalState::getInstance()));

    return 0;
}

