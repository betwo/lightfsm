/// HEADER
#include "moveit_motion.h"
/// COMPONENT
#include "global_state.h"
#include <sbc15_msgs/MoveManipulatorResult.h>

MoveitMotion::MoveitMotion(State* parent, int retries ):
    State(parent),
    event_done(this,"Arm positioned"),
    event_timeout(this,"Did not reach goal in time."),
    event_failure(this,"Faliure"),
    event_planning_failed (this,"Planning Failed"),
    event_servo_control_failed(this,"Servo Control failed"),
    retries_(retries),
    started_(false),
    takeGlobalStateGoal_(true),
    client_("arm_plan_move_server",true)
{
}

MoveitMotion::MoveitMotion(State *parent, int retries, const ArmGoal &goal):
    State(parent),
    event_done(this,"Arm positioned"),
    event_timeout(this,"Did not reach goal in time."),
    event_failure(this,"Faliure"),
    event_planning_failed (this,"Planning Failed"),
    event_servo_control_failed(this,"Servo Control failed"),
    retries_(retries),
    started_(false),
    takeGlobalStateGoal_(false),
    client_("arm_plan_move_server",true),
    constantGoal_(goal)
{

}


void MoveitMotion::entryAction()
{
    started_ = false;
    retries_left_ = retries_;
    if(takeGlobalStateGoal_)
    {
        goal_.position.resize(3);
        goal_.position[0] = GlobalState::getInstance().getCurrentArmGoal().x;
        goal_.position[1] = GlobalState::getInstance().getCurrentArmGoal().y;
        goal_.position[2] = GlobalState::getInstance().getCurrentArmGoal().z;
        goal_.pitch = GlobalState::getInstance().getCurrentArmGoal().pitch;
        goal_.yaw = GlobalState::getInstance().getCurrentArmGoal().yaw;
    }
    else
    {
        goal_.position.resize(3);
        goal_.position[0] = constantGoal_.x;
        goal_.position[1] = constantGoal_.y;
        goal_.position[2] = constantGoal_.z;
        goal_.pitch = constantGoal_.pitch;
        goal_.yaw = constantGoal_.yaw;
    }
}

void MoveitMotion::iteration()
{
    if(!started_ && retries_left_ > 0 && GlobalState::getInstance().getCurrentArmGoal().valid)
    {
        started_ = true;
        --retries_left_;
        client_.sendGoal(goal_,boost::bind(&MoveitMotion::doneCb, this, _1, _2));
    }

}


void MoveitMotion::doneCb(const actionlib::SimpleClientGoalState& /*state*/,
                          const sbc15_msgs::MoveManipulatorResultConstPtr &result)
{
    if(result->error_code == sbc15_msgs::MoveManipulatorResult::SUCCESS) {
        std::cout << "Pose reached" << std::endl;
        GlobalState::getInstance().setCurrentArmGoalInvalid();
        event_done.trigger();
    } else {
        switch(result->error_code)
        {
        case sbc15_msgs::MoveManipulatorResult::TIMED_OUT:
        {
            event_timeout.trigger();
            break;
        }
        case sbc15_msgs::MoveManipulatorResult::PLANNING_FAILED:
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::CONTROL_FAILED:
            event_servo_control_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::NO_IK_SOLUTION:
            ROS_WARN("NO IK Solution");
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::GOAL_IN_COLLISION:
            ROS_WARN("Goal in Collision");
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::GOAL_CONSTRAINTS_VIOLATED:
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::FRAME_TRANSFORM_FAILURE:
            ROS_ERROR("Can not transform frame");
            event_failure.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::COLLISION_CHECKING_UNAVAILABLE:
            ROS_ERROR("Collsion checking unavailible. Check moveit configuration.");
            event_failure.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::GOAL_VIOLATES_PATH_CONSTRAINTS:
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::INVALID_GOAL_CONSTRAINTS:
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::INVALID_GROUP_NAME:
            ROS_ERROR("Invalid group name. Flawed moveit configuration?");
            event_failure.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::INVALID_LINK_NAME:
            ROS_ERROR("Invalid link name. Flawed moveit configuration?");
            break;
        case sbc15_msgs::MoveManipulatorResult::INVALID_MOTION_PLAN:
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::INVALID_OBJECT_NAME:
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::INVALID_ROBOT_STATE:
            ROS_WARN("Invalid robot state.");
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            ROS_WARN("Enviroment changed. Leading to invalid motion plan.");
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::PREEMPTED:
            event_failure.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::ROBOT_STATE_STALE:
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::SENSOR_INFO_STALE:
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::START_STATE_VIOLATES_PATH_CONSTRAINTS:
            ROS_WARN("Start state violates path constraints. May change path constraints.");
            event_planning_failed.trigger();
            break;
        case sbc15_msgs::MoveManipulatorResult::UNABLE_TO_AQUIRE_SENSOR_DATA:
            ROS_WARN("No sensor data aquired");
            event_planning_failed.trigger();
            break;
        default:
            event_failure.trigger();
            break;
        }
        started_ = false;
    }

}
