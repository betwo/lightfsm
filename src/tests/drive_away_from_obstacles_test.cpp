/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../states/drive_away_from_obstacles.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sick14_state_machine_node_drive_away_from_obstacles_test");
    ros::NodeHandle nh;

    sick14_fsm_global::waitForRosTime();

    // STATES
    DriveAwayFromObstacles drive_away(State::NO_PARENT, 0.25, 1.5);

    // ACTIONS
    drive_away.action_entry.push_back(Action(boost::bind(&sick14_fsm_global::action::say, "Driving away from obstacles")));


    StateMachine state_machine(&drive_away);

    state_machine.run(boost::bind(&GlobalState::update, &GlobalState::getInstance()));

    return 0;
}

