/// HEADER
#include "gripper_state.h"

/// COMPONENT
#include <sbc15_msgs/GripperServices.h>
#include <sbc15_msgs/Object.h>
#include "../states/global_state.h"

GripperState::GripperState(State* parent, int type,double effort)
    : State(parent),
      event_done(this,"Gripper positioned"),
      type_(type),
      effort_(effort)
{
    gripper_client_ = GlobalState::getInstance().nh.serviceClient<sbc15_msgs::GripperServices>("/gripper_services");
    cup_effort_ = GlobalState::getInstance().nh.param("cup_effort",0.2);
    bat_effort_ = GlobalState::getInstance().nh.param("battery_effort",0.9);
}

void GripperState::entryAction()
{
    sbc15_msgs::ObjectPtr obj =GlobalState::getInstance().getCurrentObject();
    if(obj->type == sbc15_msgs::Object::OBJECT_CUP)
    {
        effort_ = cup_effort_;
    }
    if(obj->type == sbc15_msgs::Object::OBJECT_BATTERY)
    {
        effort_ = bat_effort_;
    }

}

void GripperState::iteration()
{
    sbc15_msgs::GripperServices msgs;
    msgs.request.action = type_;
    msgs.request.effort = effort_;
    gripper_client_.call(msgs.request,msgs.response);

    event_done.trigger();
}
