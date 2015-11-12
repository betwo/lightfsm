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
#include "states/select_task.h"
#include "states/goto_object.h"
#include "states/wait.h"
#include "states/explore.h"
#include "states/fetch_object.h"
#include "states/back_up.h"
#include "states/go_to_base.h"


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
 
    ROS_WARN("waiting for ros time");
    sbc15_fsm_global::waitForRosTime();
    
    // STATES
    ROS_INFO("creating states");

    WaitForGoSignal init(State::NO_PARENT);
    Wait error(State::NO_PARENT, 2);
    Wait goal(State::NO_PARENT, 2);
    Wait done_loop(State::NO_PARENT, 30);

    SelectTask select(State::NO_PARENT);

    Explore explore(State::NO_PARENT);

    FetchObject fetch_object(State::NO_PARENT, true);

    GoToBase goto_base(State::NO_PARENT);

    // ACTIONS
    ROS_INFO("connecting events");
    init.event_done >> select;

    select.event_object_selected >> fetch_object;
    select.event_object_unknown >> explore;
    select.event_all_objects_collected >> goto_base;

    goto_base.event_base_unknown >> explore;

    fetch_object.event_object_unknown >> select;
    fetch_object.event_failure >> select;
    fetch_object.event_object_fetched >> select;

    explore.event_object_found >> select;

    goal.event_done >> done_loop;
    done_loop.event_done >> done_loop;

    // TRANSITIONS
    error.event_done >> goal;

    // TALKING
    init.action_entry << boost::bind(&sbc15_fsm_global::action::say, "Waiting for go signal!");
    init.action_exit << boost::bind(&sbc15_fsm_global::action::say, "It's show time!");
    explore.action_entry << boost::bind(&sbc15_fsm_global::action::say, "Exploring the environment.");
    fetch_object.goto_object.action_entry << boost::bind(&sbc15_fsm_global::action::say, "Going to the object");
    fetch_object.pickup_object.action_entry << boost::bind(&sbc15_fsm_global::action::say, "Collecting the object");

    ROS_INFO("starting state machine");
    StateMachine state_machine(&init);

    ros::Publisher state_pub = nh.advertise<std_msgs::String>("fsm_state", 1);

    ros::Time last_pub = ros::Time(0);
    ros::Duration state_pub_rate(1.0);

    state_machine.run([&](State* current_state) {
        tick(current_state);

        ros::Time now = ros::Time::now();
        if(now > last_pub + state_pub_rate) {
            last_pub = now;
            std_msgs::String msg;
            msg.data = state_machine.generateGraphDescription();
            state_pub.publish(msg);
        }
    });

    return 0;
}

