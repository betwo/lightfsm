/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../states/drive_to_wall.h"
#include "../states/wait.h"
#include "../states/back_up.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sick14_state_machine_node_map_exploration_test");
    ros::NodeHandle nh;

    sick14_fsm_global::waitForRosTime();

    // STATES
    DriveToWall drive_to_wall(State::NO_PARENT, 0.10, 1.0);
    Wait wait_at_wall(State::NO_PARENT, 2.0, true);
    BackUp drive_back_from_wall(State::NO_PARENT, 1.0, 0.5);
    Quit error(State::NO_PARENT);
    Quit quit(State::NO_PARENT);

    // ACTIONS
    drive_to_wall.action_entry << boost::bind(&sick14_fsm_global::action::say, "Testing Driving to Wall");
    error.action_entry << boost::bind(&sick14_fsm_global::action::say, "Abort, distance is too high!");
    quit.action_entry << boost::bind(&sick14_fsm_global::action::say, "At Target, Have a nice day!");

    // TRANSITIONS
    drive_to_wall.event_positioned >> wait_at_wall;
    drive_to_wall.event_distance_to_high >> error;
    wait_at_wall.event_done >> drive_back_from_wall;
    drive_back_from_wall.event_positioned >> drive_to_wall;

    StateMachine state_machine(&drive_to_wall);

    state_machine.run(boost::bind(&GlobalState::update, &GlobalState::getInstance()));

    return 0;
}

