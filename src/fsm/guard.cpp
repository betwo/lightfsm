/// HEADER
#include "guard.h"

/// SYSTEM
#include <boost/lambda/lambda.hpp>

Guard::Guard() : condition_(boost::lambda::constant(true))
{
}

Guard::Guard(Condition condition) : condition_(condition)
{
}

bool Guard::evaluate() const
{
    return condition_();
}
