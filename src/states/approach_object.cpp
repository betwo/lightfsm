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
      observe_time_(5.0),
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
                GlobalState& global = GlobalState::getInstance();

                tf::Transform trafo = global.getTransform("/odom", o->header.frame_id, o->header.stamp, ros::Duration(0.5));
                tf::Pose object_odom = trafo * pose;

                sbc15_msgs::ObjectPtr o_odom = boost::make_shared<sbc15_msgs::Object>(*o);
                o_odom->header.frame_id = "/odom";
                tf::poseTFToMsg(object_odom, o_odom->pose);
                objects_.push_back(o_odom);
            }
        }

        ros::Time now = ros::Time::now();
        while(!objects_.empty() && objects_.front()->header.stamp + keep_time_ < now) {
            objects_.pop_front();
        }
    };

    sub_objects = global.nh.subscribe<sbc15_msgs::Object>("/objects", 100, cb);

    start_time_ = ros::Time::now();

    sbc15_msgs::ObjectPtr current_object = global.getCurrentObject();
    type = current_object->type;
    tf::poseMsgToTF(current_object->pose, start_pose_);

    last_error_pos_ = std::numeric_limits<double>::infinity();
    last_error_ang_ = std::numeric_limits<double>::infinity();
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
        sbc15_msgs::ObjectConstPtr target = objects_.back();

        position(target);
    }
}

void ApproachObject::position(const sbc15_msgs::ObjectConstPtr& object_odom)
{
    GlobalState& global = GlobalState::getInstance();

    tf::Stamped<tf::Pose> pose_object_odom;
    tf::poseMsgToTF(object_odom->pose, pose_object_odom);

    tf::Transform odom_to_base_link = global.getTransform("/base_link", "/odom");
    tf::Pose object_base_link = odom_to_base_link * pose_object_odom;

    ROS_INFO_STREAM("going to object @" << object_base_link.getOrigin().x() << "\t / " << object_base_link.getOrigin().y());

    switch(type) {
    case sbc15_msgs::Object::OBJECT_CUP:
    {
        tf::Vector3 object_pos = object_base_link.getOrigin();
        tf::Vector3 desired_pos = object_pos - distance_ * (object_pos.normalized());

        tf::Pose error(tf::createQuaternionFromYaw(std::atan2(desired_pos.y(), desired_pos.x())), desired_pos);
        double yaw = tf::getYaw(error.getRotation());

        ROS_INFO_STREAM("error is " << error.getOrigin().x() << "\t / " << error.getOrigin().y() << ", yaw: " << yaw);

        visualization_msgs::Marker marker = global.makeMarker(1.0, 0.0, 0.0, "error", 0);
        marker.header.frame_id = "/base_link";
        geometry_msgs::Point start, end;
        end.x = error.getOrigin().x();
        end.y = error.getOrigin().y();
        marker.points.push_back(start);
        marker.points.push_back(end);
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.025;
        marker.scale.y = 0.075;
        marker.scale.z = 0;
        global.mark(marker);


        geometry_msgs::Twist twist;

        double e = error.getOrigin().length();

        bool pos_good = e < 0.01 || (e < 0.07 && e > last_error_pos_);
        bool ang_good = std::abs(yaw) < 0.01 || (yaw < 0.04 && yaw > last_error_ang_);

        if(pos_good && ang_good) {
            event_approached.trigger();

        } else {
            double ex = error.getOrigin().x();
            double ey = error.getOrigin().y();
            twist.linear.x = ex * 0.6;
            twist.linear.y = ey * 0.6;
            double speed = hypot(twist.linear.x, twist.linear.y);

            double max_speed = 0.1;
            double min_speed = 0.005;

            if(speed > max_speed) {
                twist.linear.x *= max_speed / speed;
                twist.linear.y *= max_speed / speed;
            } else if(speed < min_speed) {
                twist.linear.x *= min_speed / speed;
                twist.linear.y *= min_speed / speed;
            }
            twist.angular.z =  yaw * 0.5;
        }

        last_error_pos_ = e;
        last_error_ang_ = yaw;

        GlobalState::getInstance().move(twist);
    }
        break;

    case sbc15_msgs::Object::OBJECT_BATTERY:
    {
        tf::Quaternion rot_map = tf::createQuaternionFromYaw(tf::getYaw(object_base_link.getRotation()));

    }
        break;

    default:
        event_failure.trigger();
    }
}
