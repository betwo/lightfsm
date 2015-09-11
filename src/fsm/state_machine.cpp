/// HEADER
#include "state_machine.h"

/// COMPONENT
#include "transition.h"

/// SYSTEM
#include <ros/ros.h>
#include <std_msgs/Bool.h>

StateMachine::StateMachine(State* initial_state)
    : state_(initial_state)
{
    state_->performEntryAction();
}

namespace {
void kill_sub(const std_msgs::BoolConstPtr& /*kill*/, bool& k) {
    k = true;
}
}

void StateMachine::run(boost::function<void(State*)> callback)
{
    ROS_INFO_STREAM("starting withstate " << state_->getName());

    check();

    bool kill = false;

    ros::NodeHandle pnh("~");
    ros::Subscriber sub = pnh.subscribe<std_msgs::Bool>("kill", 1, boost::bind(&kill_sub, _1, kill));


    while(ros::ok()) {
        callback(state_);
        bool terminal = !step();

        if(terminal) {
            return;
        }

        if(kill) {
            return;
        }

        // handle ros stuff
        ros::spinOnce();
        state_->getRate().sleep();
    }
}

bool StateMachine::step()
{
    std::vector<const Transition*> possible_transitions;

    state_->tick(possible_transitions);

    // can we perform a transition?
    if(!possible_transitions.empty()) {
        if(possible_transitions.size() > 1) {
            std::cerr << possible_transitions.size() << " possible transitions, using the first one!" << std::endl;
        }
        perform(*possible_transitions.front());

    } else if(state_->isTerminal()) {
        state_->performExitAction();
        return false;
    }

    return true;
}


void StateMachine::check()
{

}

void StateMachine::perform(const Transition& transition)
{
    State* next_state = transition.getTarget();

    ROS_INFO_STREAM("switching from state " << state_->getName() << " to " << next_state->getName());

    state_->performExitAction();
    transition.getEvent()->performTransitionActions();
    transition.performAction();
    state_ = next_state;
    state_->performEntryAction();
}
