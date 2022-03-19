/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm_ros/state_machine_ros_executor.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../utils/map_explorer.h"
#include "../states/place_cup.h"

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
    PlaceCup placeCup(State::NO_PARENT);

    // ACTIONS
    using sbc15_fsm_global::action::say;
    placeCup.action_entry << []() { say("Testing placement of the cup."); };
    placeCup.event_cup_placed << []() { say("The blue cup has been placed."); };

    StateMachine state_machine(&placeCup);
    StateMachineRosExecutor executor(state_machine);

    executor.run(std::bind(&tick, std::placeholders::_1));

    return 0;
}
