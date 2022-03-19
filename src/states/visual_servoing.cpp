/// HEADER
#include "visual_servoing.h"

/// COMPONENT
#include "global_state.h"

VisualServoing::VisualServoing(State* parent, int retries)
  : State(parent)
  , event_done(this, "Arm in Position Ready to Grab.")
  , event_timeout(this, "More Time Needed")
  , event_failure(this, "failed")
  , event_out_of_range(this, "Object is out of range")
  , event_servo_control_failed(this, "Controll of Arm failed")
  , event_no_object(this, "No object Visible")
  , retries_(retries)
  , started_(false)
  ,

  client_("servoingActionController", true)
{
    event_done << [this]() {
        GlobalState& global = GlobalState::getInstance();

        int type = global.getCurrentObject()->type;

        std::string talk = "Ready to grasp.";
        switch (type) {
            case sbc15_msgs::Object::OBJECT_CUP:
                talk = "Cup " + talk;
                break;
            case sbc15_msgs::Object::OBJECT_BATTERY:
                talk = "Battery " + talk;
                break;
        }

        global.talk(talk);
        global.setObjectCollected(type);
    };
}

void VisualServoing::entryAction()
{
    retries_left_ = retries_;
    started_ = false;
    goal_.timeout = ros::Duration(120);
    goal_.phi = 0;
    goal_.theta = 0;
    goal_.object = GlobalState::getInstance().getCurrentObject()->type;
}

void VisualServoing::iteration()
{
    if (!started_ && retries_left_ > 0) {
        started_ = true;
        --retries_left_;
        client_.sendGoal(goal_, std::bind(&VisualServoing::doneCb, this, std::placeholders::_1, std::placeholders::_2));
    }
}

void VisualServoing::doneCb(const actionlib::SimpleClientGoalState& /*state*/,
                            const sbc15_msgs::visual_servoingResultConstPtr& result)
{
    if (result->error_code == sbc15_msgs::visual_servoingResult::SUCCESS) {
        std::cout << "Object is grasped" << std::endl;
        event_done.trigger();
    } else {
        switch (result->error_code) {
            case sbc15_msgs::visual_servoingResult::TARGET_OUT_OF_RANGE: {
                double dist = result->distance;
                GlobalState::getInstance().setDesiredDistance(dist);
                event_out_of_range.trigger();
                break;
            }
            case sbc15_msgs::visual_servoingResult::SERVO_CONTROLL_FAILD:
                event_servo_control_failed.trigger();
                break;
            case sbc15_msgs::visual_servoingResult::TIME_OUT:
                event_timeout.trigger();
                break;
            case sbc15_msgs::visual_servoingResult::NO_IK_SOLUTION:
                event_failure.trigger();
                break;
            case sbc15_msgs::visual_servoingResult::NO_OBJECT_VISIBLE:
                event_no_object.trigger();
                break;
            default:
                event_failure.trigger();
                break;
        }
        started_ = false;
    }
}
