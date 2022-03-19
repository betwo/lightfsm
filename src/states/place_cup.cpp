#include "place_cup.h"
#include "global_state.h"
#include <sbc15_msgs/GripperServices.h>
#include <sbc15_msgs/PreplannedTrajectories.h>
#include "../global.h"

ArmGoal tmp()  // TODO remove
{
    ArmGoal res;
    res.x = 0.418;
    res.y = 0;
    res.z = -0.177;
    res.pitch = 1.5708;
    res.yaw = 0;
    return res;
}
ArmGoal tmp2()  // TODO remove
{
    ArmGoal res;
    res.x = 0.106;
    res.y = 0;
    res.z = 0.27;
    res.pitch = 1.5708;
    res.yaw = 0;
    return res;
}

PlaceCup::PlaceCup(State* parent)
  : MetaState(parent)
  , goToBase(this)
  , goToCup(this, "pickCup1")
  , closeGripper(this, sbc15_msgs::GripperServices::Request::GRAB, 0.6)
  , takeCup(this, "pickCup2")
  , placeCup(this, 1, 0.08, 0.08)
  , openGri(this, sbc15_msgs::GripperServices::Request::RESET_GRIPPER, 0)
  , goToCrane(this, 1, tmp2())
  , goToRest(this, sbc15_msgs::PreplannedTrajectories::Request::PLACE_ARM_FROM_FRONT, 1)
  , event_cup_placed(this, "Cup is placed")
  , event_failure(this, "Failure")

{
    sbc15_msgs::ObjectPtr obj(new sbc15_msgs::Object);
    obj->type = sbc15_msgs::Object::OBJECT_CUP;
    GlobalState::getInstance().setCurrentObject(obj);

    using sbc15_fsm_global::action::say;

    // Success
    event_entry_meta >> goToBase;

    goToBase.event_done >> goToCup;
    goToCup.event_done >> closeGripper;
    closeGripper.event_done >> takeCup;
    takeCup.event_done >> placeCup;
    placeCup.event_done << []() { say("Cup is placed."); };
    placeCup.event_done >> openGri;
    openGri.event_done >> goToCrane;
    goToCrane.event_done >> goToRest;

    goToRest.event_done >> event_cup_placed;

    // failures
    goToBase.event_failure >> event_failure;

    placeCup.event_failure >> event_failure;
    placeCup.event_planning_failed >> goToBase;
    placeCup.event_servo_control_failed >> placeCup;

    goToCup.event_failure >> event_failure;
    takeCup.event_failure >> event_failure;

    goToCrane.event_planning_failed >> goToCrane;
    goToCrane.event_servo_control_failed >> goToCrane;
    goToCrane.event_failure >> event_failure;

    goToRest.event_failure >> goToRest;
}

// void PlaceCup::entryAction()
//{
//    sbc15_msgs::ObjectPtr obj(new sbc15_msgs::Object);
//    obj->type = sbc15_msgs::Object::OBJECT_CUP;
//    GlobalState::getInstance().setCurrentObject(obj);
//}
