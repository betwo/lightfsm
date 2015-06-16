/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../states/get_cube.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sick14_state_machine_node_map_exploration_test");
    ros::NodeHandle nh;

    sick14_fsm_global::waitForRosTime();

    // STATES
    GetCube get_cube(State::NO_PARENT);
    Quit okay(State::NO_PARENT);
    Quit quit(State::NO_PARENT);

    // ACTIONS
    get_cube.position.action_entry << boost::bind(&sick14_fsm_global::action::say, "Testing Getting a cube");
    get_cube.signal_on.action_entry << boost::bind(&sick14_fsm_global::action::say, "Please give me a cube!");
    get_cube.eval_cube.event_no_cube << boost::bind(&sick14_fsm_global::action::say, "Did not get a cube!");
    okay.action_entry << boost::bind(&sick14_fsm_global::action::say, "All done, Have a nice day!");
    quit.action_entry << boost::bind(&sick14_fsm_global::action::say, "No Cube!");

    // TRANSITIONS
    get_cube.back_up.event_positioned >> get_cube.position;

    StateMachine state_machine(&get_cube.signal_on);

    state_machine.run(boost::bind(&GlobalState::update, &GlobalState::getInstance()));

    return 0;
}

