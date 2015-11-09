/// HEADER
#include "approach_object.h"

/// COMPONENT
#include "global_state.h"

ApproachObject::ApproachObject(State *parent, double distance, double velocity)
    : State(parent),
      event_approached(this, "approached the object"),
      event_orientation_mismatch(this, "the orientation of the object is wrong"),
      event_failure(this, "approach failed"),

      distance_(distance), velocity_(velocity),
      observe_time_(10.0),
      keep_time_(observe_time_.toSec() + 5.0)
{

}

void ApproachObject::entryAction()
{
    GlobalState& global = GlobalState::getInstance();

    boost::function<void(const sbc15_msgs::ObjectConstPtr&)> cb =
            [this](const sbc15_msgs::ObjectConstPtr& o)
    {
        if(o->type == type) {

            tf::Stamped<tf::Pose> pose;
            tf::poseMsgToTF(o->pose, pose);

            double d = (pose.getOrigin() - start_pose_.getOrigin()).length();
            if(d < distance_ / 2.0) {
                objects_.push_back(pose);
            }
        }

        ros::Time now = ros::Time::now();
        while(!objects_.empty() && objects_.front().stamp_ + keep_time_ < now) {
            objects_.pop_front();
        }
    };

    sub_objects = global.nh.subscribe<sbc15_msgs::Object>("/objects", 100, cb);

    start_time_ = ros::Time::now();

    sbc15_msgs::ObjectPtr current_object = global.getCurrentObject();
    type = current_object->type;
    tf::poseMsgToTF(current_object->pose, start_pose_);
}

void ApproachObject::exitAction()
{
    sub_objects = ros::Subscriber();
}

void ApproachObject::iteration()
{
    GlobalState& global = GlobalState::getInstance();

    ros::Time now = ros::Time::now();
    if(start_time_ + observe_time_ > now) {
        return;
    }

    if(objects_.empty()) {
        event_failure.trigger();

    } else {
        tf::Stamped<tf::Transform> target = objects_.back();

        position(target, type);
    }
}

void ApproachObject::position(const tf::Stamped<tf::Transform>& object_pose, int type)
{
    switch(type) {
    case sbc15_msgs::Object::OBJECT_CUP:
    {
    }
        break;

    case sbc15_msgs::Object::OBJECT_BATTERY:
    {
    }
        break;

    default:
        event_failure.trigger();
    }
}
