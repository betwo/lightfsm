/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../utils/map_explorer.h"
#include "../states/wait.h"
#include "../states/explore.h"

void tick(State* current_state)
{
    GlobalState::getInstance().update(current_state);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sbc15_state_machine_node",
              ros::InitOption::NoSigintHandler);
    ros::NodeHandle nh;

    sbc15_fsm_global::waitForRosTime();

    // STATES
    Explore explore(State::NO_PARENT);
    Wait goal(State::NO_PARENT, 10.0);

    // ACTIONS
    using sbc15_fsm_global::action::say;
    explore.action_entry << [](){ say("Testing Map Exploration."); };
    explore.event_object_found >> goal;
    explore.event_object_found << [](){ say("The blue cup has been found."); };

    goal.event_done >> goal;

    StateMachine state_machine(&explore);

    state_machine.run(std::bind(&tick, std::placeholders::_1));

    return 0;
}

