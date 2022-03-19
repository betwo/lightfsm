#ifndef WAIT_FOR_GO_SIGNAL_H
#define WAIT_FOR_GO_SIGNAL_H

/// COMPONENT
#include "lightfsm/state.h"
#include "lightfsm/triggered_event.h"

/// SYSTEM
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

class WaitForGoSignal : public State
{
public:
    TriggeredEvent event_done;

public:
    WaitForGoSignal(State* parent);

protected:
    void entryAction();
    void iteration();

    double desiredFrequency() const;

private:
    void cmdReceived(const std_msgs::StringConstPtr& cmd);
    void joystickReceived(const sensor_msgs::Joy::ConstPtr& joy);

private:
    ros::Subscriber sub_joy_;
    ros::Subscriber sub_cmd_;
};

#endif  // WAIT_FOR_GO_SIGNAL_H
