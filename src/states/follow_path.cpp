/// HEADER
#include "follow_path.h"

/// COMPONENT
#include "global_state.h"

FollowPath::FollowPath(State *parent, int retries)
    : State(parent),
      event_positioned(this, "positioned at target"),
      event_error(this, "error happened"),

      retries_(retries)
{
}

void FollowPath::entryAction()
{
    retries_left_ = retries_;
}

void FollowPath::iteration()
{

}

void FollowPath::doneCb(const actionlib::SimpleClientGoalState& /*state*/,
                        const path_msgs::NavigateToGoalResultConstPtr& result)
{
    if(result->status == path_msgs::NavigateToGoalResult::STATUS_SUCCESS) {
        std::cout << "path successfully finished" << std::endl;
        event_positioned.trigger();
    } else {
        if(retries_left_ > 0) {
            GlobalState::getInstance().moveTo(goal_, std::bind(&FollowPath::doneCb, this, std::placeholders::_1, std::placeholders::_2));
            --retries_left_;
        } else {
            std::cout << "path not finished" << std::endl;
            event_error.trigger();
        }
    }
}

void FollowPath::moveTo(const tf::Pose &target, TriggeredEvent &event)
{
    geometry_msgs::PoseStamped msg;
    tf::poseTFToMsg(target, msg.pose);
    moveTo(msg, event);
}

void FollowPath::moveTo(const tf::Pose &target, TriggeredEvent &event, std::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> cb)
{
    geometry_msgs::PoseStamped msg;
    tf::poseTFToMsg(target, msg.pose);
    moveTo(msg, event, cb);
}

void FollowPath::moveTo(const geometry_msgs::PoseStamped &target_msg, TriggeredEvent& event,
                        std::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> cb)
{
    goal_ = target_msg;
    GlobalState::getInstance().moveTo(goal_, std::bind(&FollowPath::doneCb, this, std::placeholders::_1, std::placeholders::_2), cb);
    event.trigger();
}

void FollowPath::moveTo(const geometry_msgs::PoseStamped &target_msg, TriggeredEvent& event)
{
    goal_ = target_msg;
    GlobalState::getInstance().moveTo(goal_, std::bind(&FollowPath::doneCb, this, std::placeholders::_1, std::placeholders::_2));
    event.trigger();
}
