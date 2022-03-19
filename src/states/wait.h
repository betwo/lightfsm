#ifndef WAIT_H
#define WAIT_H

/// COMPONENT
#include "lightfsm/state.h"
#include "lightfsm/triggered_event.h"

/// SYSTEM
#include <ros/time.h>

class Wait : public State
{
public:
    TriggeredEvent event_done;

public:
    Wait(State* parent, double duration);

protected:
    void entryAction();
    void iteration();

    double desiredFrequency() const;

private:
    double duration_;
    ros::Time continue_at_;
};

#endif  // WAIT_H
