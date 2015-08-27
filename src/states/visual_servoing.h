#ifndef VISUAL_SERVOING_H
#define VISUAL_SERVOING_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

#include <actionlib/client/simple_action_client.h>
#include <sbc15_msgs/visual_servoingAction.h>

class VisualServoing: public State
{
public:
    TriggeredEvent event_object_gripped;
    TriggeredEvent event_failure;
public:
    VisualServoing(State* parent);

    void entryAction();
    void iteration();

private:
    actionlib::SimpleActionClient<sbc15_msgs::visual_servoingAction> ac_;

};

#endif // VISUAL_SERVOING_H
