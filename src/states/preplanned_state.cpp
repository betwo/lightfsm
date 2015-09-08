/// HEADER
#include "preplanned_state.h"

/// COMPONENT
#include <sbc15_msgs/PreplannedTrajectories.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include "../states/global_state.h"

PreplannedState::PreplannedState(State* parent, int type, int retries)
    : State(parent),
      event_done(this,"Arm positioned"),
      event_failure(this, "error happend"),
      type_(type),
      retries_(retries),
      started_(false)
{
    planedTrajectoryClient_ = GlobalState::getInstance().nh.serviceClient<sbc15_msgs::PreplannedTrajectories>("preplanned_trajectories");
}

void PreplannedState::entryAction()
{
    retries_left_ = retries_;
    started_ = false;
}

void PreplannedState::iteration()
{

    if(!started_ && retries_left_ > 0)
    {
        started_ = true;
        --retries_left_;
        sbc15_msgs::PreplannedTrajectories msgs;
        msgs.request.trajectory = type_;
        planedTrajectoryClient_.call(msgs.request,msgs.response);
//        if(msgs.response.result.error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
//        {
            event_done.trigger();
//        }
//        else
//        {
//            if(retries_left_ < 0)
//            {
//                event_failure.trigger();
//            }
//        }
        started_ = false;
    }
}
