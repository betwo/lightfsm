/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../utils/map_explorer.h"
#include "../states/wait.h"
#include "../states/explore.h"
#include "../states/fetch_object.h"
#include "../states/select_task.h"

void tick(State* current_state)
{
    GlobalState::getInstance().update(current_state);
}

struct WaitForGo: public State
{
public:
    TriggeredEvent event_go;

public:
    WaitForGo(State* parent)
        : State(parent), event_go(this, "Go Signal")
    {
        sub = GlobalState::getInstance().nh.subscribe<std_msgs::Bool>("/go", 1, boost::bind(&WaitForGo::go, this, _1));
    }

    void go(const std_msgs::BoolConstPtr&)
    {
        event_go.trigger();
    }

    void iteration()
    {

    }

private:
    ros::Subscriber sub;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sbc15_state_machine_node_map_exploration_test");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    sbc15_fsm_global::waitForRosTime();

    bool store = p_nh.param("store", true);

    // STATES
    WaitForGo wait(State::NO_PARENT);
    Wait goal(State::NO_PARENT, 10.0);

    SelectTask select(State::NO_PARENT);

    Explore explore(State::NO_PARENT);
    FetchObject get_object(State::NO_PARENT, store);

    // ACTIONS
    explore.action_entry.push_back(Action(boost::bind(&sbc15_fsm_global::action::say, "Testing Map Exploration.")));
    explore.event_object_found >> select;
    explore.event_object_found << []() {
        GlobalState& global = GlobalState::getInstance();
        global.talk("An object has been found");
    };

    select.event_object_selected >> get_object;
    select.event_object_unknown >> explore;
    select.event_all_objects_collected >> goal;

    goal.event_done >> goal;

    StateMachine state_machine(&wait);

    state_machine.run(boost::bind(&tick, _1));

    return 0;
}

