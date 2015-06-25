/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../utils/map_explorer.h"
#include "../states/wait.h"

void tick(State* current_state)
{
    GlobalState::getInstance().update(current_state);
}


class Explore : public State
{
public:
    TriggeredEvent event_object_found;

public:
    Explore(State* parent)
        : State(parent),
          event_object_found(this, "A cup has been found")
    {
    }

    void entryAction()
    {
        explorer_.startExploring();
    }

    void iteration()
    {
        if(!explorer_.isExploring()) {
            ROS_INFO("start exploring");
            explorer_.startExploring();
        }

        auto objects = GlobalState::getInstance().getObjects();
        ROS_INFO_STREAM_THROTTLE(1, "there are " << objects.size() << " objects mapped");
        for(const sbc15_msgs::Object& o : objects) {
            if(o.type == sbc15_msgs::Object::OBJECT_CUP) {
                event_object_found.trigger();
            }
        }
    }

    void exitAction()
    {
        explorer_.stopExploring();
    }

protected:
    double desiredFrequency() const
    {
        return 1.0;
    }

private:
    MapExplorer explorer_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sbc15_state_machine_node_map_exploration_test");
    ros::NodeHandle nh;

    sbc15_fsm_global::waitForRosTime();

    // STATES
    Explore explore(State::NO_PARENT);
    Wait goal(State::NO_PARENT, 10.0);

    // ACTIONS
    explore.action_entry.push_back(Action(boost::bind(&sbc15_fsm_global::action::say, "Testing Map Exploration.")));
    explore.event_object_found >> goal;
    explore.event_object_found.connect(Action(boost::bind(&sbc15_fsm_global::action::say, "The blue cup has been found-")));

    goal.event_done >> goal;

    StateMachine state_machine(&explore);

    state_machine.run(boost::bind(&tick, _1));

    return 0;
}

