#ifndef APPROACH_OBJECT_H
#define APPROACH_OBJECT_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"

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

    void position(const tf::Stamped<tf::Transform>& object_pose, int object_type);

private:
    ros::Subscriber sub_objects;

    double distance_;
    double velocity_;

    ros::Time start_time_;
    tf::Pose start_pose_;

    int type;
    std::deque< tf::Stamped<tf::Pose> > objects_;

    ros::Duration observe_time_;
    ros::Duration keep_time_;
};

#endif // APPROACH_OBJECT_H
