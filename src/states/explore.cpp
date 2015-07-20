/// HEADER
#include "explore.h"

/// COMPONENT
#include "../states/global_state.h"


Explore::Explore(State* parent)
    : State(parent),
      event_object_found(this, "A cup has been found")
{
}

void Explore::entryAction()
{
    explorer_.startExploring();
}

void Explore::iteration()
{
    if(!explorer_.isExploring()) {
        ROS_INFO("start exploring");
        explorer_.startExploring();
    }

    auto objects = GlobalState::getInstance().getObjects();
    ROS_INFO_STREAM_THROTTLE(1, "there are " << objects.size() << " objects mapped");
    for(const sbc15_msgs::Object& o : objects) {
        if(o.type == sbc15_msgs::Object::OBJECT_CUP) {
            event_object_found.trigger();
        }
    }
}

void Explore::exitAction()
{
    explorer_.stopExploring();
}

double Explore::desiredFrequency() const
{
    return 1.0;
}
