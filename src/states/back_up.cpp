/// HEADER
#include "back_up.h"

/// COMPONENT
#include "global_state.h"

BackUp::BackUp(State *parent, double distance, double velocity)
    : State(parent),
      event_positioned(this, "backed up"),
      distance_(distance), velocity_(velocity)
{
}

void BackUp::entryAction()
{
    start_pose_ = GlobalState::getInstance().pose;
}

void BackUp::iteration()
{
    tf::Vector3 start = start_pose_.getOrigin();
    tf::Vector3 now = GlobalState::getInstance().pose.getOrigin();

    geometry_msgs::Twist cmd;
    if((now - start).length() >= distance_) {
        event_positioned.trigger();
    } else {
        cmd.linear.x = velocity_;
    }
    GlobalState::getInstance().move(cmd);
}
