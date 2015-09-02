#include "store_object.h"
/// COMPONENT
#include "../states/global_state.h"
#include <sbc15_msgs/PreplannedTrajectories.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

StoreObject::StoreObject(State *parent, int retries):
    State(parent),
    object_stored(this,"object is stored"),
    event_failure(this,"error occured"),
    retries_(retries),
    started_(false)

{
    planedTrajectoryClient_ = GlobalState::getInstance().nh.serviceClient<sbc15_msgs::PreplannedTrajectories>("preplanned_trajectories");
}

void StoreObject::entryAction()
{
    retries_left_ = retries_;
    started_ = true;
}

void StoreObject::iteration()
{
    if(!started_ && retries_left_ > 0)
    {
        started_ = true;
        --retries_left_;
        sbc15_msgs::PreplannedTrajectories msgs;
        msgs.request.trajecory = sbc15_msgs::PreplannedTrajectories::Request::PLACE_CUP;
        planedTrajectoryClient_.call(msgs.request,msgs.response);

        if(msgs.response.result.error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
        {
            object_stored.trigger();
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
