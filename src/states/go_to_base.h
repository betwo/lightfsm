#ifndef GO_TO_BASE_H
#define GO_TO_BASE_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
#include <sbc15_msgs/GripperServices.h>
#include "ros/ros.h"

class GoToBase : public State
{
public:
    TriggeredEvent event_done;
    TriggeredEvent event_base_unknown;

public:
    GoToBase(State* parent);

    void entryAction();
    void iteration();
};

#endif  // GO_TO_BASE_H
