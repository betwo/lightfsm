/// HEADER
#include "approach_object.h"

/// COMPONENT
#include "global_state.h"

namespace {
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
}

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

    std::function<void(const sbc15_msgs::ObjectConstPtr&)> cb =
            [this](const sbc15_msgs::ObjectConstPtr& o)
    {
        if(o->type == type) {
            GlobalState& global = GlobalState::getInstance();

            tf::Stamped<tf::Pose> pose;
            tf::poseMsgToTF(o->pose, pose);

            tf::Transform trafo_to_base_link;
            auto it = frame_to_base_link.find(o->header.frame_id);
            if(it == frame_to_base_link.end()) {
                trafo_to_base_link = global.getTransform("/arm_base_link", o->header.frame_id, o->header.stamp, ros::Duration(0.5));
                frame_to_base_link[o->header.frame_id] = trafo_to_base_link;

            } else {
                trafo_to_base_link = it->second;
            }

            tf::Pose object_base_link = trafo_to_base_link * pose;

            tf::Vector3 pos = object_base_link.getOrigin();
            if(pos.z() > -0.35 && pos.z() < 0.35)  {
                tf::Vector3 z_up(0, 0, 1);
                tf::Vector3 z_obj = tf::quatRotate(object_base_link.getRotation(), z_up);

                double angle = std::abs(z_up.angle(z_obj));
                double max_angle = type == sbc15_msgs::Object::OBJECT_CUP ? M_PI / 2 : M_PI/4;
                if((angle < max_angle) || (M_PI - angle) < max_angle) {

                    double d = pose.getOrigin().length();
                    if(d < distance_ * 5) {

                        tf::Transform trafo_to_odom = global.getTransform("/odom", "/arm_base_link", o->header.stamp, ros::Duration(0.5));
                        tf::Pose object_odom = trafo_to_odom * object_base_link;

                        tf::Vector3 error = (start_pose_odom_.getOrigin() - object_odom.getOrigin());

                        if(error.length() < 0.5) {
                            sbc15_msgs::ObjectPtr o_odom = boost::make_shared<sbc15_msgs::Object>(*o);
                            o_odom->header.frame_id = "/odom";
                            tf::poseTFToMsg(object_odom, o_odom->pose);
                            objects_.push_back(o_odom);
                            ROS_INFO_STREAM("seeing object");

                        } else {
                            ROS_INFO_STREAM("drop object @ " << object_odom.getOrigin().x() << " / " <<
                                            object_odom.getOrigin().y() << " with start pose " <<
                                            start_pose_odom_.getOrigin().x() << " / " <<
                                            start_pose_odom_.getOrigin().y() <<
                                            " with offset " << error.x() << " / " << error.y());
                        }

                    } else {
                        ROS_INFO_STREAM("drop object  @ " << pose.getOrigin().x() << " / " <<
                                        pose.getOrigin().y() << " with start pose " <<
                                        start_pose_odom_.getOrigin().x() << " / " <<
                                        start_pose_odom_.getOrigin().y() << " with distance " << d);
                    }
                } else {
                    ROS_INFO_STREAM("drop object with angle " << angle);
                }
            } else {
                ROS_INFO_STREAM("drop object with height " << pos.z());
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
    tf::poseMsgToTF(current_object->pose, start_pose_odom_);

    tf::Transform trafo_to_odom = global.getTransform("/odom", current_object->header.frame_id, current_object->header.stamp, ros::Duration(2.0));
    start_pose_odom_ = trafo_to_odom * start_pose_odom_;

    last_error_pos_ = std::numeric_limits<double>::infinity();
    last_error_ang_ = std::numeric_limits<double>::infinity();


}

void ApproachObject::exitAction()
{
    sub_objects = ros::Subscriber();
}

void ApproachObject::iteration()
{
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

    tf::Transform odom_to_base_link = global.getTransform("/arm_base_link", "/odom");
    tf::Pose object_base_link = odom_to_base_link * pose_object_odom;

    tf::Vector3 pos = object_base_link.getOrigin();

    double normXY = hypot(pos.x(),pos.y());
    pos -= 0.14 * tf::Vector3(pos.x()/normXY,pos.y()/normXY,0);

    global.setCurrentArmGoal(pos.x(),pos.y(),pos.z(), M_PI_2, std::atan2(pos.y(), pos.x()));

    ROS_INFO_STREAM("going to object @" << object_base_link.getOrigin().x() << "\t / " << object_base_link.getOrigin().y());

    double d = distance_;

    switch(type) {
    case sbc15_msgs::Object::OBJECT_CUP:
    {
        double dp = d + 0.04;

        tf::Vector3 object_pos = object_base_link.getOrigin();
        tf::Vector3 desired_pos = object_pos - dp * (object_pos.normalized());

        tf::Pose error = tf::Pose(tf::createQuaternionFromYaw(std::atan2(desired_pos.y(), desired_pos.x())), desired_pos);

        driveToPose(error);
        break;
    }

    case sbc15_msgs::Object::OBJECT_BATTERY:
    {
        double dp = d + 0.1;

        double obj_yaw = tf::getYaw(object_base_link.getRotation());

        tf::Pose desired_pose_a(tf::createQuaternionFromYaw(0), tf::Vector3(dp, 0.0, 0.0));
        tf::Pose error_a = object_base_link * desired_pose_a.inverse();
        error_a.setRotation(tf::createQuaternionFromYaw(obj_yaw));

        tf::Pose desired_pose_b(tf::createQuaternionFromYaw(0), tf::Vector3(-dp, 0.0, 0.0));
        tf::Pose error_b = object_base_link * desired_pose_b.inverse();
        error_b.setRotation(tf::createQuaternionFromYaw(obj_yaw + M_PI));

        tf::Pose error;
        if(error_a.getOrigin().length() < error_b.getOrigin().length()) {
            error = error_a;
        } else {
            error = error_b;
        }

        // TODO: call  event_orientation_mismatch !

        visualization_msgs::Marker orientation_marker = global.makeMarker(0.0, 1.0, 0.0, "error", 1);
        orientation_marker.header.frame_id = "/arm_base_link";
        orientation_marker.pose.position.x = error.getOrigin().x();
        orientation_marker.pose.position.y = error.getOrigin().y();
        tf::quaternionTFToMsg(error.getRotation(), orientation_marker.pose.orientation);
        orientation_marker.type = visualization_msgs::Marker::ARROW;
        orientation_marker.scale.x = 0.5;
        orientation_marker.scale.y = 0.075;
        orientation_marker.scale.z = 0.075;
        global.mark(orientation_marker);

        error.setRotation(tf::createQuaternionFromYaw(std::atan2(error.getOrigin().y(), error.getOrigin()   .x())));

        driveToPose(error);
    }
        break;

    default:
        event_failure.trigger();
        return;
    }
}


void ApproachObject::driveToPose(const tf::Pose &error)
{
    geometry_msgs::Twist twist;
    double error_yaw = tf::getYaw(error.getRotation());

    double e = std::hypot(error.getOrigin().x(), error.getOrigin().y());

    bool pos_good = e < 0.01 || (e < 0.07 && e > last_error_pos_);
    bool ang_good = std::abs(error_yaw) < M_PI / 4 || (std::abs(error_yaw) < M_PI / 2 && error_yaw > last_error_ang_);

    ROS_INFO_STREAM("pos error: " << e << "\tangular error: " << error_yaw);

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
        twist.angular.z =  error_yaw * 0.5;
    }

    last_error_pos_ = e;
    last_error_ang_ = error_yaw;

    GlobalState& global = GlobalState::getInstance();

    global.move(twist);

    visualization_msgs::Marker error_marker = global.makeMarker(1.0, 0.0, 0.0, "error", 0);
    error_marker.header.frame_id = "/arm_base_link";
    geometry_msgs::Point start, end;
    end.x = error.getOrigin().x();
    end.y = error.getOrigin().y();
    error_marker.points.push_back(start);
    error_marker.points.push_back(end);
    error_marker.type = visualization_msgs::Marker::ARROW;
    error_marker.scale.x = 0.025;
    error_marker.scale.y = 0.075;
    error_marker.scale.z = 0;
    global.mark(error_marker);


    ROS_INFO_STREAM("error is " << error.getOrigin().x() << "\t / " << error.getOrigin().y() << ", yaw: " << error_yaw);
}
