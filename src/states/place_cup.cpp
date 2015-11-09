#include "place_cup.h"
#include "global_state.h"
#include <sbc15_msgs/GripperServices.h>
#include <sbc15_msgs/PreplannedTrajectories.h>
#include "../global.h"

ArmGoal tmp() //TODO remove
{
    ArmGoal res;
    res.x = 0.418;
    res.y = 0;
    res.z = -0.177;
    res.pitch = 1.5708;
    res.yaw = 0;
    return res;
}
ArmGoal tmp2() //TODO remove
{
    ArmGoal res;
    res.x = 0.106;
    res.y = 0;
    res.z = 0.27;
    res.pitch = 1.5708;
    res.yaw = 0;
    return res;
}


PlaceCup::PlaceCup(State* parent):
    MetaState(parent),
     goToBase(this),
     prePos(this,sbc15_msgs::PreplannedTrajectories::Request::START_ARM,1),      // TODO: Replace with take cup from storage
     preVs(this,1,tmp()),                                                        // TODO: remove
     visualServo(this,1),                                                        // TODO: remove
     planToCrane(this,1,tmp2()),                                                 // TODO: remove
     planToCrane2(this,1,tmp2()),
     planToCrane3(this,1,tmp2()),
     placeCup(this,1,0.08,0.08),
     openGri(this,sbc15_msgs::GripperServices::Request::RESET_GRIPPER),
     openGri2(this,sbc15_msgs::GripperServices::Request::RESET_GRIPPER),
     goToRest(this,sbc15_msgs::PreplannedTrajectories::Request::PLACE_ARM_FROM_FRONT,1),
     event_cup_placed(this,"Cup is placed"),
     event_failure(this,"Failure")

{
    sbc15_msgs::ObjectPtr obj(new sbc15_msgs::Object);
    obj->type = sbc15_msgs::Object::OBJECT_CUP;
    GlobalState::getInstance().setCurrentObject(obj);
    // Success
    event_entry_meta >> goToBase;

    goToBase.event_done >> prePos;
    prePos.event_done >> openGri;
    openGri.event_done >> preVs;
    preVs.event_done << boost::bind(&sbc15_fsm_global::action::say, "Starting Visual Servoing.");
    preVs.event_done >> visualServo;
    preVs.event_timeout << boost::bind(&sbc15_fsm_global::action::say, "Starting Visual Servoing.");
    preVs.event_timeout >> visualServo;
    visualServo.event_object_gripped << boost::bind(&sbc15_fsm_global::action::say, "Cup grabbed.");
    visualServo.event_object_gripped >> planToCrane;
    planToCrane.event_done >> placeCup;
    planToCrane.event_timeout >>placeCup;
    placeCup.event_done << boost::bind(&sbc15_fsm_global::action::say, "Cup is placed.");
    placeCup.event_timeout >> openGri2;
    placeCup.event_done >> openGri2;
    openGri2.event_done >> planToCrane2;
    planToCrane2.event_done >> goToRest;
    planToCrane2.event_timeout >> goToRest;
    goToRest.event_done >> event_cup_placed;


    //failures
    goToBase.event_failure >> event_failure;

    placeCup.event_failure >> event_failure;
    placeCup.event_planning_failed >> goToBase;
    placeCup.event_servo_control_failed >>placeCup;

    planToCrane.event_planning_failed >> planToCrane;
    planToCrane.event_servo_control_failed >> planToCrane;
    planToCrane.event_failure >> event_failure;

    goToRest.event_failure >> goToRest;


    //TODO remove
    prePos.event_failure >> event_failure;
    preVs.event_failure >>event_failure;
    preVs.event_planning_failed >>event_failure;





}
