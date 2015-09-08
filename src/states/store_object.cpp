#include "store_object.h"
/// COMPONENT
#include "../states/global_state.h"
#include <sbc15_msgs/PreplannedTrajectories.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

StoreObject::StoreObject(State *parent, int retries):
    MetaState(parent),
    object_stored(this,"object is stored"),
    event_failure(this,"error occured"),

    place_object(this, sbc15_msgs::PreplannedTrajectoriesRequest::PLACE_CUP /* or 2 for battery... */, retries),
    open_gripper(this, sbc15_msgs::GripperServices::Request::OPEN_GRIPPER),
    rest_position(this, sbc15_msgs::PreplannedTrajectoriesRequest::PLACE_ARM_FROM_CUP  /* or 6 for battery... */, retries)

{
    event_entry_meta >> place_object;

    place_object.event_done >> open_gripper;
    place_object.event_failure >> place_object;

    open_gripper.event_done >> rest_position;

    rest_position.event_done >> object_stored;
    rest_position.event_failure >> rest_position;
}
