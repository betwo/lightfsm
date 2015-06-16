/// HEADER
#include "global.h"

void sick14_fsm_global::action::print(const std::string& str)
{
    ROS_WARN_STREAM("action: " << str);
}

void sick14_fsm_global::action::say(const std::string& str)
{
    GlobalState::getInstance().talk(str);
}

void sick14_fsm_global::waitForRosTime()
{
    ros::WallRate wait(60);
    while(ros::Time::now().toNSec() == 0) {
        ros::spinOnce();
        wait.sleep();
    }
}


Initial::Initial(State* parent)
    : State(parent)
{
}


Error::Error(State* parent)
    : State(parent)
{
}

bool Error::isTerminal() const
{
    return true;
}

Quit::Quit(State* parent)
    : State(parent)
{
}

bool Quit::isTerminal() const
{
    return true;
}
