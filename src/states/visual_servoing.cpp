#include "visual_servoing.h"

VisualServoing::VisualServoing(State* parent):
    State(parent),
    event_object_gripped(this,"gripped object"),
    event_failure(this,"failed")
{
}

void VisualServoing::entryAction()
{
    //TODO
}

void VisualServoing::iteration()
{

}
