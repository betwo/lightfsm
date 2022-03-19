/// HEADER
#include "wait_for_go_signal.h"

/// PROJECT
#include "global_state.h"

/// SYSTEM
#include <sound_play/SoundRequest.h>

WaitForGoSignal::WaitForGoSignal(State* parent) : State(parent), event_done(this, "done waiting")
{
    sub_joy_ = GlobalState::getInstance().private_nh.subscribe<sensor_msgs::Joy>(
        "/joy", 10, std::bind(&WaitForGoSignal::joystickReceived, this, std::placeholders::_1));
    sub_cmd_ = GlobalState::getInstance().private_nh.subscribe<std_msgs::String>(
        "/command", 1, std::bind(&WaitForGoSignal::cmdReceived, this, std::placeholders::_1));
}

void WaitForGoSignal::entryAction()
{
}

void WaitForGoSignal::iteration()
{
}

double WaitForGoSignal::desiredFrequency() const
{
    return 5.0;
}

void WaitForGoSignal::joystickReceived(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (joy->buttons[1] && joy->buttons[2]) {
        event_done.trigger();
    }
}

void WaitForGoSignal::cmdReceived(const std_msgs::StringConstPtr& cmd)
{
    ROS_INFO_STREAM("command received: " << cmd->data);
    if (cmd->data == "start") {
        event_done.trigger();
    }
}
