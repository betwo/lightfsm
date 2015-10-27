/// HEADER
#include "wait.h"

#include "global_state.h"

/// SYSTEM
#include <ros/console.h>

Wait::Wait(State* parent, double duration)
    : State(parent),
      event_done(this, "done waiting"),
      duration_(duration)
{
}

void Wait::entryAction()
{
    continue_at_ = ros::Time::now() + ros::Duration(duration_);
    ROS_INFO_STREAM("waiting for " << duration_ << " seconds until " << continue_at_);
}

void Wait::iteration()
{
//    GlobalState& global = GlobalState::getInstance();

    ros::Time now = ros::Time::now();
    if(now >= continue_at_) {
        event_done.trigger();
    }
}

double Wait::desiredFrequency() const
{
    return 20.0;
}
