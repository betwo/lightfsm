#ifndef PLAN_ARM_MOTION_H
#define PLAN_ARM_MOTION_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

class PlanArmMotion: public State
{
public:
    TriggeredEvent event_at_goal;
    TriggeredEvent event_failure;
public:
    PlanArmMotion(State* parent);

    void entryAction();
    void iteration();
};

#endif // PLAN_ARM_MOTION_H
