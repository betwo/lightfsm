#include "visual_servoing.h"

VisualServoing::VisualServoing(State* parent):
    State(parent),
    event_object_gripped(this,"gripped object"),
    event_failure(this,"failed"),
    ac_("servoingActionController", true)
{
}

void VisualServoing::entryAction()
{
    ac_.waitForServer(); //will wait for infinite time
    //TODO
}

void VisualServoing::iteration()
{

}
