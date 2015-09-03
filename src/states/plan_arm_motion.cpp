#include "plan_arm_motion.h"
/// COMPONENT
#include "../states/global_state.h"
#include <sbc15_msgs/PreplannedTrajectories.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

PlanArmMotion::PlanArmMotion(State* parent, int retries):
    State(parent),
    event_at_goal(this,"Arm positioned at target pose"),
    event_failure(this, "error happend"),
    retries_(retries),
    started_(false)
{
    planedTrajectoryClient_ = GlobalState::getInstance().nh.serviceClient<sbc15_msgs::PreplannedTrajectories>("preplanned_trajectories");
}

void PlanArmMotion::entryAction()
{
    retries_left_ = retries_;
    started_ = false;
}

void PlanArmMotion::iteration()
{

    if(!started_ && retries_left_ > 0)
    {
        started_ = true;
        --retries_left_;
        sbc15_msgs::PreplannedTrajectories msgs;
        msgs.request.trajectory = sbc15_msgs::PreplannedTrajectories::Request::PICK_CUP;
        planedTrajectoryClient_.call(msgs.request,msgs.response);

        if(msgs.response.result.error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
        {
            event_at_goal.trigger();
        }
        else
        {
            if(retries_left_ < 0)
            {
                event_failure.trigger();
            }
        }
        started_ = false;
    }
}
