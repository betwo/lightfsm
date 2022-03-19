#ifndef GOTOOBJECT_H
#define GOTOOBJECT_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "follow_path.h"

/// PROJECT
#include <sbc15_msgs/Object.h>

class GoToObject : public MetaState
{
public:
    TriggeredEvent event_object_reached;
    TriggeredEvent event_object_unknown;

    TriggeredEvent event_sent_goal;
    TriggeredEvent event_path_failure;

    FollowPath follow_path;

public:
    GoToObject(State* parent, double offset);

    void entryAction();
    void iteration();

protected:
    double desiredFrequency() const;

private:
    sbc15_msgs::Object target_;

    double offset_;
};

#endif  // GOTOOBJECT_H
