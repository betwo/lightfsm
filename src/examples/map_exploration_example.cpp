/// COMPONENT
#include "lightfsm/state_machine.h"
#include "lightfsm_ros/state_machine_ros_executor.h"
#include "lightfsm/state.h"
#include "lightfsm/meta_state.h"
#include "lightfsm/event.h"
#include "lightfsm/triggered_event.h"

#include "../global.h"

#include "../utils/map_explorer.h"
#include "../states/wait.h"
#include "../states/explore.h"

void tick(State* current_state)
{
    GlobalState::getInstance().update(current_state);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sbc15_state_machine_node", ros::InitOption::NoSigintHandler);
    ros::NodeHandle nh;

    sbc15_fsm_global::waitForRosTime();

    // STATES
    Explore explore(State::NO_PARENT);
    Wait goal(State::NO_PARENT, 10.0);

    // ACTIONS
    using sbc15_fsm_global::action::say;
    explore.action_entry << []() { say("Testing Map Exploration."); };
    explore.event_object_found >> goal;
    explore.event_object_found << []() { say("The blue cup has been found."); };

    goal.event_done >> goal;

    StateMachine state_machine(&explore);

    StateMachineRosExecutor executor(state_machine);

    executor.run(std::bind(&tick, std::placeholders::_1));

    return 0;
}
