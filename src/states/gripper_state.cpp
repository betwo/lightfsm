/// HEADER
#include "gripper_state.h"

/// COMPONENT
#include <sbc15_msgs/GripperServices.h>
#include <sbc15_msgs/Object.h>
#include "../states/global_state.h"

GripperState::GripperState(State* parent, int type, double effort)
  : State(parent), event_done(this, "Gripper positioned"), type_(type), effort_(effort)
{
    gripper_client_ = GlobalState::getInstance().nh.serviceClient<sbc15_msgs::GripperServices>("/gripper_services");
}

void GripperState::entryAction()
{
}

void GripperState::iteration()
{
    sbc15_msgs::GripperServices msgs;
    msgs.request.action = type_;
    msgs.request.effort = effort_;
    gripper_client_.call(msgs.request, msgs.response);

    event_done.trigger();
}
