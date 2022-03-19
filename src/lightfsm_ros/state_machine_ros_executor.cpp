/// HEADER
#include "lightfsm_ros/state_machine_ros_executor.h"

/// COMPONENT
#include "lightfsm/transition.h"
#include "lightfsm/meta_state.h"

/// SYSTEM
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <std_msgs/String.h>

StateMachineRosExecutor::StateMachineRosExecutor(StateMachine& state_machine) : StateMachineExecutor(state_machine)
{
}

namespace
{
void kill_sub(const std_msgs::BoolConstPtr& /*kill*/, bool& k)
{
    k = true;
}
}  // namespace

void StateMachineRosExecutor::run(std::function<void(State*)> callback)
{
    State* initial_state = state_machine_.getState();
    ROS_INFO_STREAM("starting with state " << initial_state->getName());
    initial_state->performEntryAction();

    bool kill = false;

    ros::NodeHandle pnh("~");
    ros::Subscriber sub = pnh.subscribe<std_msgs::Bool>("kill", 1, boost::bind(&kill_sub, _1, kill));

    while (ros::ok()) {
        callback(state_machine_.getState());
        state_machine_.step();

        if (state_machine_.getState()->isTerminal()) {
            state_machine_.shutdown();
            return;
        }

        if (kill) {
            state_machine_.shutdown();
            return;
        }

        // handle ros stuff
        ros::spinOnce();
        ros::Rate sleep_rate(state_machine_.getState()->desiredFrequency());
        sleep_rate.sleep();
    }
}
