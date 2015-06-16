/// COMPONENT
#include "../fsm/state_machine.h"
#include "../fsm/state.h"
#include "../fsm/meta_state.h"
#include "../fsm/event.h"
#include "../fsm/triggered_event.h"

#include "../global.h"

#include "../states/global_state.h"
#include "../states/position_to_target.h"
#include "../states/orient_to_sign.h"
#include "../states/drive_to_wall.h"
#include "../states/back_up.h"


class BlockTest : public State
{
public:
    BlockTest(State* parent)
        : State(parent)
    {

    }

    double desiredFrequency() const
    {
        return 1.0;
    }

    void entryAction()
    {
    }

    void iteration()
    {
        GlobalState::getInstance().checkCollectionAreas();
    }

    void exitAction()
    {
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sick14_state_machine_node_blocked_station_test");
    ros::NodeHandle nh;

    sick14_fsm_global::waitForRosTime();


    GlobalState::getInstance().current_number = 1;

    // STATES
    BlockTest init(State::NO_PARENT);
    StateMachine state_machine(&init);

    ROS_INFO("running machine");
    state_machine.run(boost::bind(&GlobalState::update, &GlobalState::getInstance()));

    return 0;
}

