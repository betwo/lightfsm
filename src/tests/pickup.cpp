/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../states/select_task.h"
#include "../states/goto_object.h"
#include "../states/wait.h"
#include "../states/explore.h"
#include "../states/fetch_object.h"
#include "../states/back_up.h"

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
        : State(parent), event_object_found(this, "Object found"),
          obj_(new sbc15_msgs::Object)
    {
        subObject_ = GlobalState::getInstance().nh.subscribe<sbc15_msgs::Object>("/leia/pickup_object",1,boost::bind(&WaitForObject::objectCb,this,_1));
    }

    double desiredFrequency() const override
    {
        return 1.0;
    }


    void iteration()
    {
        //sbc15_msgs::ObjectPtr o(new sbc15_msgs::Object);
        //o->type = sbc15_msgs::Object::OBJECT_BATTERY;
        //ros::spinOnce();
        //if(found_)
        //{
        //    GlobalState::getInstance().setCurrentObject(obj_);

          //  event_object_found.trigger();
        //}
    }
private:
    ros::Subscriber subObject_;
    sbc15_msgs::ObjectPtr obj_;
    //bool found_;

    void objectCb(sbc15_msgs::ObjectConstPtr object)
    {
        ROS_WARN("object detected");
        if(object->type == sbc15_msgs::Object::OBJECT_BATTERY || object->type == sbc15_msgs::Object::OBJECT_CUP)
        {
            obj_->type = object->type;
            obj_->detection_type = object->detection_type;
            obj_->header.frame_id = object->header.frame_id;
            obj_->header.stamp = ros::Time::now();
            obj_->pose = object->pose;
            //found_ = true;
            GlobalState::getInstance().setCurrentObject(obj_);
            event_object_found.trigger();
        }
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sbc15_state_machine_node",
              ros::InitOption::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    sbc15_fsm_global::waitForRosTime();

    bool store = p_nh.param("store", true);
    double x = p_nh.param("prePlannedPosX",0.223); // -0.006, 0.309
    double y = p_nh.param("prePlannedPosY",-0.006);
    double z = p_nh.param("prePlannedPosZ",0.309);
    double pitch = p_nh.param("prePlannedPosePitch",M_PI_2);
    ArmGoal interimPose;
    interimPose.valid = true;
    interimPose.x = x;
    interimPose.y = y;
    interimPose.z = z;
    interimPose.pitch = pitch;

    // STATES
    WaitForObject wait_for_object(State::NO_PARENT);
    PickupObject pickup_object(State::NO_PARENT, store,interimPose);
    Wait goal(State::NO_PARENT, 10.0);

    // ACTION
    wait_for_object.event_object_found >> pickup_object;
    pickup_object.event_object_pickedup >> wait_for_object;

    StateMachine state_machine(&wait_for_object);

    state_machine.run(boost::bind(&tick, _1));

    return 0;
}


