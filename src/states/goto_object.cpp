/// HEADER
#include "goto_object.h"

/// COMPONENT
#include "../states/global_state.h"


GoToObject::GoToObject(State* parent)
    : MetaState(parent),
      event_object_reached(this, "The object has been reached"),
      event_object_unknown(this, "The object pose is not known"),
      event_sent_goal(this, "Sent a waypoint"),
      event_path_failure(this, "failure: cannot reach the object"),

      follow_path(this, 4)
{
    event_sent_goal >> follow_path;

    follow_path.event_positioned >> event_object_reached;
    follow_path.event_error >> event_path_failure;
}

void GoToObject::entryAction()
{
    const double offset = 0.45 /*m*/;

    auto objects = GlobalState::getInstance().getObjects();
    ROS_INFO_STREAM_THROTTLE(1, "there are " << objects.size() << " objects mapped");
    for(const sbc15_msgs::Object& o : objects) {
        if(o.type == sbc15_msgs::Object::OBJECT_CUP) {
            target_ = o;

            tf::Pose object_map;
            tf::poseMsgToTF(target_.pose, object_map);

            tf::Pose robot_map = GlobalState::getInstance().pose;

            tf::Vector3 delta = (robot_map.getOrigin() - object_map.getOrigin());

            tf::Vector3 pos_map = object_map.getOrigin() + delta * offset / delta.length();
            tf::Quaternion rot_map = tf::createQuaternionFromYaw(std::atan2(-delta.y(), -delta.x()));

            tf::Pose object_offset_map(rot_map, pos_map);

            follow_path.moveTo(object_offset_map, event_sent_goal);
            return;
        }

        if(o.type == sbc15_msgs::Object::OBJECT_BATTERY) {
            target_ = o;

            tf::Pose object_map;
            tf::poseMsgToTF(target_.pose, object_map);

            tf::Pose robot_map = GlobalState::getInstance().pose;

            tf::Quaternion rot_map = tf::createQuaternionFromYaw(tf::getYaw(object_map.getRotation()));

            tf::Vector3 delta_obj(offset, 0, 0);
            tf::Vector3 delta_world = tf::quatRotate(rot_map, delta_obj);

            tf::Vector3 pos_map_a = object_map.getOrigin() + delta_world;
            tf::Vector3 pos_map_b = object_map.getOrigin() + delta_world;

            double dist_a = (pos_map_a - robot_map.getOrigin()).length();
            double dist_b = (pos_map_b - robot_map.getOrigin()).length();

            tf::Vector3 pos_map = (dist_a < dist_b) ? pos_map_a : pos_map_b;

            tf::Pose object_offset_map(rot_map, pos_map);

            follow_path.moveTo(object_offset_map, event_sent_goal);
            return;
        }
    }

    event_object_unknown.trigger();
}

void GoToObject::iteration()
{
}

double GoToObject::desiredFrequency() const
{
    return 1.0;
}
