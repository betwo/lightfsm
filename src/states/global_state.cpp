/// HEADER
#include "global_state.h"

/// PROJECT
#include <sbc15_msgs/GetObjects.h>
#include "../fsm/state.h"

/// SYSTEM
#include <geometry_msgs/PointStamped.h>
#include <sound_play/SoundRequest.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

using namespace path_msgs;

GlobalState::GlobalState()
    : private_nh("~"),
      client_("navigate_to_goal", true),
      tfl_(ros::Duration(15.0))
{
    //    pub_speech_ = nh.advertise<std_msgs::String> ("/speech", 10, true);
    pub_move_ = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10, true);
    pub_move_unsafe_ = nh.advertise<geometry_msgs::Twist> ("/cmd_vel_unsafe", 10, true);
    pub_marker_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 100, true);
    pub_sound_ = nh.advertise<sound_play::SoundRequest>("/robotsound", 1, true);
    pub_state_ = private_nh.advertise<std_msgs::String>("state", 100, true);


    pub_velo_ = nh.advertise<std_msgs::Float32>("/velocity", 1, true);

    boost::function<void(const std_msgs::Float32ConstPtr&)> set_velo = [&](const std_msgs::Float32ConstPtr& msg){
        desired_speed_ = std::max(0.1f, std::min(2.0f, msg->data));
        std::cerr << "updating velocity to " << desired_speed_ << std::endl;
        nh.setParam("desired_speed", desired_speed_);
        publishVelocity();
    };
    sub_set_velocity_ = nh.subscribe<std_msgs::Float32>("/set_velocity", 10, set_velo);

    client_objects_ = nh.serviceClient<sbc15_msgs::GetObjects>("/get_objects");

    nh.param<double>("desired_speed", desired_speed_, 0.2);
    publishVelocity();

    //    client_.waitForServer();
    //tfl_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(2.0));
}

GlobalState& GlobalState::getInstance()
{
    static GlobalState s;
    return s;
}

tf::Transform GlobalState::getTransform(const std::string &from, const std::string &to, const ros::Time &time, const ros::Duration& wait)
{
    assert(!from.empty());
    assert(!to.empty());

    tf::StampedTransform t;
    try {
        if(tfl_.waitForTransform(from, to, time, wait)) {
            tfl_.lookupTransform(from, to, time, t);
            return t;

        } else {
            ROS_ERROR_STREAM("cannot lookup transform from " << from << " to " << to << " at time " << time);
        }

    } catch(const tf2::TransformException& e) {
        ROS_ERROR_STREAM("cannot lookup transform from " << from << " to " << to << ": " << e.what());
    }
    return tf::Pose();
}

void GlobalState::update(State* current_state)
{
    //pose = getTransform("/map", "/base_link");

    std_msgs::String state;
    state.data = current_state->getName();
    pub_state_.publish(state);
}

std::vector<sbc15_msgs::Object> GlobalState::getObjects()
{
    sbc15_msgs::GetObjects srv;
    if(client_objects_.call(srv)) {
        return srv.response.objects;
    } else {
        ROS_ERROR_THROTTLE(1, "cannot get objects via /get_objects");
        return std::vector<sbc15_msgs::Object>();
    }
}

sbc15_msgs::ObjectPtr GlobalState::getCurrentObject()
{
    // refresh object pose
    if(current_object_) {
        for(const sbc15_msgs::Object& o : getObjects()) {
            if(o.type == current_object_->type) {
                current_object_ = sbc15_msgs::ObjectPtr(new sbc15_msgs::Object(o));
            }
        }
    }
    return current_object_;
}

void GlobalState::setCurrentObject(sbc15_msgs::ObjectPtr current)
{
    current_object_ = current;
}

bool GlobalState::isObjectCollected(int type)
{
    return object_collected_[type];
}

void GlobalState::setObjectCollected(int type)
{
    object_collected_[type] = true;
}

void GlobalState::setDesiredDistance(double &dist)
{
    desired_distance_ = dist;
}

double GlobalState::getDesiredDistance() const
{
    return desired_distance_;
}

double GlobalState::getDesiredVelocity() const
{
    return desired_speed_;
}

ArmGoal& GlobalState::getCurrentArmGoal()
{
    return current_arm_goal_;
}

void GlobalState::setCurrentArmGoal(double &x, double &y, double &z, double &pitch)
{
    current_arm_goal_.valid = true;
    current_arm_goal_.x = x;
    current_arm_goal_.y = y;
    current_arm_goal_.z = z;
    current_arm_goal_.pitch = pitch;
}

void GlobalState::setCurrentArmGoalInvalid()
{
    current_arm_goal_.valid = false;
}

void GlobalState::setSystemEnabled(const std::string &name, bool enabled)
{
    std::cerr << "[SBC15 State Maching] " << (enabled? "enable" : "disable") << " subsystem" << name << std::endl;
    sendSystemCommand(name, enabled ? "resume" : "stop");
}

void GlobalState::sendSystemCommand(const std::string &name, const std::string &command)
{
    if(pubs_systems_.find(name) == pubs_systems_.end()) {
        pubs_systems_.insert(std::make_pair(name, private_nh.advertise<std_msgs::String>(name + "/command", 10, true)));
    }

    std_msgs::String cmd_msg;
    cmd_msg.data = command;
    pubs_systems_[name].publish(cmd_msg);

    ros::spinOnce();
}

void GlobalState::mark(const visualization_msgs::Marker &marker)
{
    pub_marker_.publish(marker);
}

visualization_msgs::Marker GlobalState::makeMarker(float r, float g, float b, const std::string& ns, int id)
{
    visualization_msgs::Marker marker;
    marker.ns = ns;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.type = visualization_msgs::Marker::CUBE;

    return marker;
}

void GlobalState::talk(const std::string &text)
{
    sound_play::SoundRequest rq;
    rq.sound = sound_play::SoundRequest::SAY;
    rq.command = sound_play::SoundRequest::PLAY_ONCE;
    rq.arg = text;
    pub_sound_.publish(rq);
}

void GlobalState::sound(int sound)
{
    sound_play::SoundRequest rq;
    rq.sound = sound;
    rq.command = sound_play::SoundRequest::PLAY_ONCE;
    pub_sound_.publish(rq);
}

void GlobalState::move(const geometry_msgs::Twist& twist)
{
    geometry_msgs::Twist cmd = twist;
    cmd.linear.x = std::max(std::min(cmd.linear.x, 1.0), -1.0);
    cmd.linear.y = std::max(std::min(cmd.linear.y ,1.0), -1.0);
    cmd.angular.z = std::max(std::min(cmd.angular.z, 0.7), -0.7);
    pub_move_.publish(cmd);
}

void GlobalState::moveUnsafe(const geometry_msgs::Twist& twist)
{
    geometry_msgs::Twist cmd = twist;
    cmd.linear.x = std::max(std::min(cmd.linear.x, 1.0), -1.0);
    cmd.linear.y = std::max(std::min(cmd.linear.y ,1.0), -1.0);
    cmd.angular.z = std::max(std::min(cmd.angular.z, 0.7), -0.7);
    pub_move_unsafe_.publish(cmd);
}

void GlobalState::stopMoving()
{
    geometry_msgs::Twist twist;
    move(twist);
}


void GlobalState::moveTo(const tf::Pose &target,
                         boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                         int failure_mode, const std::string &planning_algorithm)
{
    moveTo(target, doneCb, boost::bind(&GlobalState::feedbackCb, this, _1), failure_mode, planning_algorithm);
}

void GlobalState::moveTo(const tf::Pose &target,
                         boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                         boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> feedbackCb,
                         int failure_mode,
                         const std::string& planning_algorithm)
{
    geometry_msgs::PoseStamped msg;
    tf::poseTFToMsg(target, msg.pose);
    moveTo(msg, doneCb, feedbackCb, failure_mode, planning_algorithm);
}


void GlobalState::moveTo(const tf::Pose &target,
                         double velocity,
                         boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                         int failure_mode, const std::string &planning_algorithm)
{
    moveTo(target, velocity, doneCb, boost::bind(&GlobalState::feedbackCb, this, _1), failure_mode, planning_algorithm);
}
void GlobalState::moveTo(const tf::Pose &target,
                         double velocity,
                         boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                         boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> feedbackCb,
                         int failure_mode,
                         const std::string& planning_algorithm)
{
    geometry_msgs::PoseStamped msg;
    tf::poseTFToMsg(target, msg.pose);
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    moveTo(msg, velocity, doneCb, feedbackCb, failure_mode, planning_algorithm);
}



void GlobalState::moveTo(const geometry_msgs::PoseStamped &target_msg,
                         boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                         int failure_mode,
                         const std::string& planning_algorithm)
{
    moveTo(target_msg, doneCb, boost::bind(&GlobalState::feedbackCb, this, _1), failure_mode, planning_algorithm);
}
void GlobalState::moveTo(const geometry_msgs::PoseStamped &target_msg,
                         boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                         boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> feedbackCb,
                         int failure_mode,
                         const std::string& planning_algorithm)
{
    moveTo(target_msg, desired_speed_, doneCb, feedbackCb, failure_mode, planning_algorithm);
}



void GlobalState::moveTo(const geometry_msgs::PoseStamped &target_msg,
                         double velocity,
                         boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                         int failure_mode,
                         const std::string& planning_algorithm)
{
    moveTo(target_msg, velocity, doneCb, boost::bind(&GlobalState::feedbackCb, this, _1), failure_mode, planning_algorithm);
}

void GlobalState::moveTo(const geometry_msgs::PoseStamped &target_msg,
                         double velocity,
                         boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                         boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> feedbackCb,
                         int failure_mode,
                         const std::string& planning_algorithm)
{
    path_msgs::NavigateToGoalGoal goal;
    goal.goal.pose = target_msg;
    goal.goal.algorithm.data = planning_algorithm;
    goal.failure_mode = failure_mode;
    goal.velocity = velocity;

    moveTo(goal, doneCb, feedbackCb);
}

void GlobalState::moveTo(const NavigateToGoalGoal &goal,
                         boost::function<void(const actionlib::SimpleClientGoalState&,const path_msgs::NavigateToGoalResultConstPtr&)> doneCb,
                         boost::function<void(const path_msgs::NavigateToGoalFeedbackConstPtr&)> feedbackCb)
{
    if(!client_.isServerConnected()) {
        ROS_WARN("waiting for navigation server");
        client_.waitForServer();
    }

    client_.sendGoal(goal,
                     doneCb,
                     boost::bind(&GlobalState::activeCb, this),
                     feedbackCb);
}



void GlobalState::activeCb()
{
}

void GlobalState::feedbackCb(const path_msgs::NavigateToGoalFeedbackConstPtr& feedback)
{
    switch (feedback->status) {
    case NavigateToGoalFeedback::STATUS_MOVING:
        ROS_DEBUG_THROTTLE(1, "Feedback: Moving");
        break;

    case NavigateToGoalFeedback::STATUS_PATH_READY:
        ROS_DEBUG("Feedback: Path is ready.");
        break;

    case NavigateToGoalFeedback::STATUS_REPLAN:
        ROS_WARN("Path is replaned.");
        break;

    case NavigateToGoalFeedback::STATUS_OBSTACLE:
        ROS_WARN("There is an obstacle.");
        break;

    case NavigateToGoalFeedback::STATUS_REPLAN_FAILED:
        ROS_ERROR("Replan failed.");
        break;

    default:
        ROS_ERROR("Feedback: Unknown status code %d", feedback->status);
        break;
    }
}

void GlobalState::publishVelocity()
{
    std_msgs::Float32 velo;
    velo.data = desired_speed_;
    pub_velo_.publish(velo);
};


