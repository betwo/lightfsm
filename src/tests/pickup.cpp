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
        : State(parent), event_object_found(this, "Object found")
    {

    }

    void iteration()
    {
        auto objects = GlobalState::getInstance().getObjects();
        ROS_INFO_STREAM_THROTTLE(1, "there are " << objects.size() << " objects mapped");
        if(!objects.empty()) {
            event_object_found.trigger();
        }
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sbc15_state_machine_node_map_exploration_test");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    sbc15_fsm_global::waitForRosTime();

    bool store = p_nh.param("store", true);

    // STATES
    WaitForObject wait_for_object(State::NO_PARENT);
    PickupObject pickup_object(State::NO_PARENT, store);
    Wait goal(State::NO_PARENT, 10.0);

    // ACTION
    wait_for_object.event_object_found >> pickup_object;
    pickup_object.event_object_pickedup >> goal;

    StateMachine state_machine(&wait_for_object);

    state_machine.run(boost::bind(&tick, _1));

    return 0;
}


