#include "go_to_base.h"


GoToBase::GoToBase(State *parent):
    State(parent),
    event_done(this,"Base Station Reached")
{
}

void GoToBase::entryAction()
{

}

void GoToBase::iteration()
{
    event_done.trigger();
}
