#include "recorded_trajectory.h"
#include "global_state.h"

RecordedTrajectory::RecordedTrajectory(State* parent, std::string trajectory)
  : State(parent)
  , event_done(this, "Goal position reached")
  , event_failure(this, "Failure")
  ,
  //       trajectory_(trajecory),
  //       client_("playAction",true),
  started_(false)

{
    goal_.path = trajectory;
}

void RecordedTrajectory::entryAction()
{
    started_ = false;
}

void RecordedTrajectory::iteration()
{
    if (!started_) {
        started_ = true;
        GlobalState::getInstance().playRecordedTrajectory(
            goal_, std::bind(&RecordedTrajectory::doneCb, this, std::placeholders::_1, std::placeholders::_2));
    }
}

void RecordedTrajectory::doneCb(const actionlib::SimpleClientGoalState& /*state*/,
                                const sbc15_msgs::PlayResultConstPtr& result)
{
    if (result->error_code == sbc15_msgs::PlayResult::SUCCESS) {
        event_done.trigger();
    } else {
        event_failure.trigger();
    }
}
