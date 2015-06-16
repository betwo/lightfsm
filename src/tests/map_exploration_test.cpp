/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../utils/map_explorer.h"


class Explore : public State
{
public:
    Explore(State* parent)
        : State(parent)
    {

    }

    void entryAction()
    {
        explorer_.startExploring();
    }

    void iteration()
    {
        if(!explorer_.isExploring()) {
            explorer_.startExploring();
        }
    }

    void exitAction()
    {
        explorer_.stopExploring();
    }

private:
    MapExplorer explorer_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sick14_state_machine_node_map_exploration_test");
    ros::NodeHandle nh;

    sick14_fsm_global::waitForRosTime();

    // STATES
    Explore explore(State::NO_PARENT);

    // ACTIONS
    explore.action_entry.push_back(Action(boost::bind(&sick14_fsm_global::action::say, "Testing Map Exploration")));


    StateMachine state_machine(&explore);

    state_machine.run(boost::bind(&GlobalState::update, &GlobalState::getInstance()));

    return 0;
}

