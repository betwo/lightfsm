#include "store_object.h"
/// COMPONENT
#include "../states/global_state.h"
#include <sbc15_msgs/PreplannedTrajectories.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

StoreObject::StoreObject(State *parent, int retries):
    MetaState(parent),
    object_stored(this,"object is stored"),
    event_failure(this,"error occured"),

    event_cup(this,"cup"),
    event_battery(this,"battery"),

    place_cup1(this, "placeCup1"),
    open_gripper_cup(this, sbc15_msgs::GripperServices::Request::OPEN_GRIPPER,0.6),
//    place_cup2(this,"placeCup2"),
    rest_position_cup(this,"sleepFromCup"),

    place_battery1(this, "placeBat1"),
    open_gripper_battery(this, sbc15_msgs::GripperServices::Request::OPEN_GRIPPER,0.6),
    place_battery2(this, "placeBat2"),
    rest_position_battery(this,"crane"),
    sleep_from_battery(this,"rest"),
    gripper_semi_close(this, sbc15_msgs::GripperServices::Request::SEMI_CLOSE,0.6)

{
    //event_entry_meta >> place_cup;

    event_cup >> place_cup1;
    place_cup1.event_done >> open_gripper_cup;
    place_cup1.event_failure >> place_cup1;

//    open_gripper_cup.event_done >> place_cup2;
    open_gripper_cup.event_done >> rest_position_cup;
//    place_cup2.event_done >> rest_position_cup;
//    place_cup2.event_failure >> place_cup2;

    rest_position_cup.event_done >> object_stored;
    rest_position_cup.event_failure >> rest_position_cup;


    event_battery >> place_battery1;
    place_battery1.event_done >> open_gripper_battery;
    place_battery1.event_failure >> place_battery1;

    open_gripper_battery.event_done >> place_battery2;
    place_battery2.event_done >> rest_position_battery;
    place_battery2.event_failure >> place_battery2;

    rest_position_battery.event_done >>sleep_from_battery;
    sleep_from_battery.event_done >> object_stored;
    sleep_from_battery.event_failure >>sleep_from_battery;
    rest_position_battery.event_failure >> rest_position_battery;
}

void StoreObject::entryAction()
{

    GlobalState& global = GlobalState::getInstance();

    sbc15_msgs::ObjectPtr o = global.getCurrentObject();

    if(o->type == sbc15_msgs::Object::OBJECT_CUP) {
        event_cup.trigger();
    } else if(o->type == sbc15_msgs::Object::OBJECT_BATTERY) {
        event_battery.trigger();
    }
}
