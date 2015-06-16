/// HEADER
#include "action.h"

Action::Action(Callable call)
    : call_(call)
{
}

Action::Action()
{

}

void Action::perform() const
{
    if(call_) {
        call_();
    }
}
