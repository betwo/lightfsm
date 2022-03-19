#include "position_to_base.h"
#include "global_state.h"

PositionToBase::PositionToBase(State* parent)
  : State(parent), event_done(this, "in position"), event_failure(this, "error happend")

{
}

void PositionToBase::entryAction()
{
    ROS_INFO("TODO IMPLEMENT;");
    GlobalState::getInstance().setCurrentArmGoal(0.5, 0.248, 0.15, 1.5708, 0.46);
    event_done.trigger();
}

void PositionToBase::iteration()
{
}
