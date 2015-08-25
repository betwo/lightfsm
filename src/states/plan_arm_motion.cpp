#include "plan_arm_motion.h"

PlanArmMotion::PlanArmMotion(State* parent):
    State(parent),
    event_at_goal(this,"Arm positioned at target pose"),
    event_failure(this, "error happend")
{
}

void PlanArmMotion::entryAction()
{

}

void PlanArmMotion::iteration()
{

}
