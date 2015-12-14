#include "rest_to_pre_position.h"

RestToPrePosition::RestToPrePosition(State* parent, int retries):
    MetaState(parent),
    event_done(this,"Arm positioned"),
    event_failure(this, "error happend"),
    start_arm(this, "awake"),
    open_gripper(this, sbc15_msgs::GripperServices::Request::OPEN_GRIPPER,0),
    pre_pos(this, retries)
{
    event_entry_meta >> start_arm;

    start_arm.event_done >> open_gripper;
    start_arm.event_failure >> event_failure;

    open_gripper.event_done >> pre_pos;

    pre_pos.event_done >> event_done;
    pre_pos.event_failure >> event_failure;
    pre_pos.event_planning_failed >> pre_pos;
    pre_pos.event_servo_control_failed >> event_failure;


}
