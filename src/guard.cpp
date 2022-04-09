/// HEADER
#include "lightfsm/guard.h"

Guard::Guard() : condition_([]() { return true; })
{
}

Guard::Guard(Condition condition) : condition_(condition)
{
}

bool Guard::evaluate() const
{
    return condition_();
}
