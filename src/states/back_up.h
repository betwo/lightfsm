#ifndef BACK_UP_H
#define BACK_UP_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

/// SYSTEM
#include <tf/tf.h>

class BackUp : public State
{
public:
    TriggeredEvent event_positioned;

public:
    BackUp(State* parent, double distance, double velocity);

    void entryAction();
    void iteration();

private:
    double distance_;
    double velocity_;
    tf::Pose start_pose_;
};

#endif  // BACK_UP_H
