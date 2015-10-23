/// COMPONENT
#include "fsm/state_machine.h"
#include "fsm/state.h"
#include "fsm/meta_state.h"
#include "fsm/event.h"
#include "fsm/triggered_event.h"

#include "global.h"

#include "states/wait_for_go_signal.h"
#include "states/global_state.h"
#include "states/wait.h"

/// SYSTEM
#include <ros/ros.h>
#include <sound_play/SoundRequest.h>

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
    WaitForGoSignal init(State::NO_PARENT);
    Wait error(State::NO_PARENT, 2);
    Error goal(State::NO_PARENT);

    // ACTIONS
    init.action_entry << boost::bind(&sbc15_fsm_global::action::say, "It's show time!");

    // TRANSITIONS
    init.event_done >> goal;
    error.event_done >> goal;

    StateMachine state_machine(&init);

    state_machine.run(boost::bind(&tick, _1));

    return 0;
}

