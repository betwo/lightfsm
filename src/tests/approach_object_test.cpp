/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../states/approach_object.h"
#include "../states/wait.h"
#include "../states/pickup_object.h"

void tick(State* current_state)
{
    GlobalState::getInstance().update(current_state);
}

struct WaitForObject : public State
{
public:
    TriggeredEvent event_object_found;

public:
    WaitForObject(State* parent)
        : State(parent), event_object_found(this, "Object found")
    {

    }

    void entryAction()
    {
        GlobalState& global = GlobalState::getInstance();

        boost::function<void(const sbc15_msgs::ObjectConstPtr&)> cb =
                [this](const sbc15_msgs::ObjectConstPtr& o)
        {
            if(o->type == sbc15_msgs::Object::OBJECT_BATTERY) {
                GlobalState& global = GlobalState::getInstance();

                tf::Transform trafo_to_base_link = global.getTransform("/arm_base_link", o->header.frame_id, o->header.stamp, ros::Duration(0.5));
                tf::Pose o_pose;
                tf::poseMsgToTF(o->pose, o_pose);
                tf::Pose in_base = trafo_to_base_link * o_pose;

                tf::Vector3 delta = in_base.getOrigin();
                double dist = delta.length();

                if(dist > 2.0 || std::abs(delta.y()) > 0.5) {
                    return;
                }

                if(in_base.getOrigin().z() > -0.6 && in_base.getOrigin().z() < 0.2) {

                    global.setCurrentObject(boost::make_shared<sbc15_msgs::Object>(*o));

                    event_object_found.trigger();
                }
            }
        };

        sub_objects = global.nh.subscribe<sbc15_msgs::Object>("/objects", 100, cb);
    }

    void exitAction()
    {
        sub_objects = ros::Subscriber();
    }

    double desiredFrequency() const override
    {
        return 1.0;
    }

    void iteration()
    {
    }

private:
    ros::Subscriber sub_objects;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sbc15_state_machine_node",
              ros::InitOption::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    sbc15_fsm_global::waitForRosTime();

    // STATES
    WaitForObject wait(State::NO_PARENT);
    Wait goal(State::NO_PARENT, 10.0);

    ApproachObject approach(State::NO_PARENT, 0.4, 0.1);
    PickupObject pickup_object(State::NO_PARENT, true);

    // ACTION
    goal.event_done >> goal;

    wait.event_object_found >> approach;

    approach.event_failure >> goal;
    approach.event_approached >> goal;
    approach.event_orientation_mismatch >> pickup_object;

    pickup_object.event_object_failure >> goal;
    pickup_object.event_object_out_of_range >> goal;
    pickup_object.event_object_pickedup >> goal;

    // TALK
    wait.action_entry << boost::bind(&sbc15_fsm_global::action::say, "Testing object approach.");
    approach.event_failure << boost::bind(&sbc15_fsm_global::action::say, "Fail.");
    approach.event_orientation_mismatch << boost::bind(&sbc15_fsm_global::action::say, "Orientation mismatch.");
    approach.event_approached << boost::bind(&sbc15_fsm_global::action::say, "Pick up!");
    pickup_object.event_object_pickedup << boost::bind(&sbc15_fsm_global::action::say, "Great success!");


    StateMachine state_machine(&wait);
    ROS_INFO("start");

    state_machine.run([&](State* current_state) {
        tick(current_state);
    });

    return 0;
}


