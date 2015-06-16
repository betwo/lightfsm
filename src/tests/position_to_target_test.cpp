/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../states/global_state.h"
#include "../states/position_to_target.h"
#include "../states/orient_to_sign.h"
#include "../states/drive_to_wall.h"
#include "../states/back_up.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sick14_state_machine_node_map_exploration_test");
    ros::NodeHandle nh;

    sick14_fsm_global::waitForRosTime();


    GlobalState::getInstance().current_number = 1;

    // STATES
    OrientToSign orient(State::NO_PARENT);
    PositionToTarget position(State::NO_PARENT);
    DriveToWall to_wall(State::NO_PARENT, 0.12, 1.0);
    BackUp backup(State::NO_PARENT, 1.0, 0.5);
    Quit quit(State::NO_PARENT);

    // ACTIONS
    orient.action_entry << boost::bind(&sick14_fsm_global::action::say, "Testing Positioning to Target");
    quit.action_entry << boost::bind(&sick14_fsm_global::action::say, "At Target, Have a nice day!");
    to_wall.event_distance_to_high << boost::bind(&sick14_fsm_global::action::say, "Aborting, distance to wall is too high!");

    // TRANSITIONS
    orient.event_positioned >> position;
    position.event_positioned >> to_wall;
    to_wall.event_positioned >> backup;
    to_wall.event_distance_to_high >> backup;
    backup.event_positioned >> position;

    StateMachine state_machine(&position);

    ROS_INFO("running machine");
    state_machine.run(boost::bind(&GlobalState::update, &GlobalState::getInstance()));

    return 0;
}

