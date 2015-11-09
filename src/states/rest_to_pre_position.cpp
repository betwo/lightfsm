#include "rest_to_pre_position.h"

RestToPrePosition::RestToPrePosition(State* parent, int retries):
    MetaState(parent),
    event_done(this,"Arm positioned"),
    event_failure(this, "error happend"),

    semi_open_gripper(this, sbc15_msgs::GripperServices::Request::SEMI_CLOSE),
    start_arm(this, sbc15_msgs::PreplannedTrajectoriesRequest::START_ARM, retries),
    open_gripper(this, sbc15_msgs::GripperServices::Request::OPEN_GRIPPER),
    //pre_pos(this, retries)
    pre_pos(this, sbc15_msgs::PreplannedTrajectories::Request::PRE_POSITION, 1)
{
    event_entry_meta >> semi_open_gripper;

    semi_open_gripper.event_done >> start_arm;

    start_arm.event_done >> open_gripper;
    start_arm.event_failure >> event_failure;

    open_gripper.event_done >> pre_pos;

    pre_pos.event_done >> event_done;
    pre_pos.event_failure >> event_failure;
//    pre_pos.event_planning_failed >> event_failure;
//    pre_pos.event_servo_control_failed >> event_failure;
//    pre_pos.event_timeout >> pre_pos;

}
