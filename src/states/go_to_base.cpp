#include "go_to_base.h"


GoToBase::GoToBase(State *parent):
    State(parent),
    event_done(this,"Base Station Reached"),
    event_base_unknown(this,"Base Station is unknown")
{
}

void GoToBase::entryAction()
{

}

void GoToBase::iteration()
{
    event_done.trigger();
}
