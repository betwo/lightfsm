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
#include "../states/fetch_object.h"

void tick(State* current_state)
{
    GlobalState::getInstance().update(current_state);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sbc15_state_machine_node_map_exploration_test");
    ros::NodeHandle nh;

    sbc15_fsm_global::waitForRosTime();

    // STATES
    Explore explore(State::NO_PARENT);
    Wait goal(State::NO_PARENT, 10.0);
    FetchObject get_cup(State::NO_PARENT);

    // ACTIONS
    explore.action_entry.push_back(Action(boost::bind(&sbc15_fsm_global::action::say, "Testing Map Exploration.")));
    explore.event_object_found >> get_cup;
    explore.event_object_found.connect(Action(boost::bind(&sbc15_fsm_global::action::say, "The blue cup has been found.")));

    goal.event_done >> goal;

    StateMachine state_machine(&explore);

    state_machine.run(boost::bind(&tick, _1));

    return 0;
}

