/// HEADER
#include "visual_servoing.h"

/// COMPONENT
#include "global_state.h"

VisualServoing::VisualServoing(State* parent,int retries):
    State(parent),
    retries_(retries),
    event_object_gripped(this,"gripped object"),
    event_failure(this,"failed"),
    client_("servoingActionController", true)
{
}

void VisualServoing::entryAction()
{
    retries_left_ = retries_;
}

void VisualServoing::iteration()
{
    client_.sendGoal(goal_,boost::bind(&VisualServoing::doneCb, this, _1, _2));

}


void VisualServoing::doneCb(const actionlib::SimpleClientGoalState& /*state*/,
                            const sbc15_msgs::visual_servoingResultConstPtr &result)
{
    if(result->error_code == sbc15_msgs::visual_servoingResult::SUCCESS) {
        std::cout << "Object is graped" << std::endl;
        event_object_gripped.trigger();
    } else {
        if(result->error_code == sbc15_msgs::visual_servoingResult::TIME_OUT && retries_left_ > 0)
        {
           iteration();
            --retries_left_;
        }
        event_failure.trigger();
    }


}


