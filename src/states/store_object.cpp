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

    place_cup(this, sbc15_msgs::PreplannedTrajectoriesRequest::PLACE_CUP, retries),
    open_gripper_cup(this, sbc15_msgs::GripperServices::Request::SEMI_CLOSE,0),
    rest_position_cup(this, sbc15_msgs::PreplannedTrajectoriesRequest::PLACE_ARM_FROM_CUP, retries),

    place_battery(this, sbc15_msgs::PreplannedTrajectoriesRequest::PLACE_BOX, retries),
    open_gripper_battery(this, sbc15_msgs::GripperServices::Request::SEMI_CLOSE,0),
    rest_position_battery(this, sbc15_msgs::PreplannedTrajectoriesRequest::PLACE_ARM_FROM_BOX, retries),
    gripper_semi_close(this, sbc15_msgs::GripperServices::Request::SEMI_CLOSE,0)

{
    //event_entry_meta >> place_cup;

    event_cup >> place_cup;
    place_cup.event_done >> open_gripper_cup;
    place_cup.event_failure >> place_cup;

    open_gripper_cup.event_done >> rest_position_cup;

    rest_position_cup.event_done >> object_stored;
    rest_position_cup.event_failure >> rest_position_cup;


    event_battery >> place_battery;
    place_battery.event_done >> open_gripper_battery;
    place_battery.event_failure >> place_battery;

    open_gripper_battery.event_done >> rest_position_battery;

    rest_position_battery.event_done >> object_stored;
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
