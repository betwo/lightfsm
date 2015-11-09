#ifndef APPROACH_OBJECT_H
#define APPROACH_OBJECT_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

/// PROJECT
#include <sbc15_msgs/Object.h>

/// SYSTEM
#include <tf/tf.h>

class ApproachObject : public State
{
public:
    TriggeredEvent event_approached;
    TriggeredEvent event_orientation_mismatch;
    TriggeredEvent event_failure;

public:
    ApproachObject(State* parent, double distance, double velocity);

    void entryAction();
    void exitAction();
    void iteration();

private:
    void position(const sbc15_msgs::ObjectConstPtr &object_odom);
    void driveToPose(tf::Pose object_base_link, double d);

private:
    ros::Subscriber sub_objects;

    double distance_;
    double velocity_;

    ros::Time start_time_;
    tf::Pose start_pose_odom_;

    int type;
    std::deque< sbc15_msgs::ObjectConstPtr > objects_;

    ros::Duration observe_time_;
    ros::Duration keep_time_;

    double last_error_pos_;
    double last_error_ang_;

    std::map<std::string, tf::Transform> frame_to_base_link;
};

#endif // APPROACH_OBJECT_H
