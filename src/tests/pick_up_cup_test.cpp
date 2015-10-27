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
    double desiredFrequency() const override
    {
        return 1.0;
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
    ros::init(argc, argv, "sbc15_state_machine_node",
              ros::InitOption::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    sbc15_fsm_global::waitForRosTime();

    bool store = p_nh.param("store", true);

    // STATES
    WaitForObject wait(State::NO_PARENT);
    Wait goal(State::NO_PARENT, 10.0);

    SelectTask select(State::NO_PARENT);

    BackUp forward(State::NO_PARENT, 1.0, 0.1);
    Explore explore(State::NO_PARENT);

    GoToObject goto_object(State::NO_PARENT);
    PickupObject pickup_object(State::NO_PARENT, store);

    // ACTION
    goal.event_done >> goal;

    wait.event_object_found >> select;

    forward.event_positioned >> explore;

    select.event_object_selected >> goto_object;
    select.event_object_unknown >> forward;
    select.event_all_objects_collected >> goal;

    goto_object.event_object_reached >> pickup_object;
    goto_object.event_object_unknown >> select;
    goto_object.event_path_failure >> goto_object;

    pickup_object.event_object_pickedup >> select;

    explore.event_object_found >> select;

    StateMachine state_machine(&select);

    state_machine.run(boost::bind(&tick, _1));

    return 0;
}


