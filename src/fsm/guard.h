#ifndef GUARD_H
#define GUARD_H

/// SYSTEM
#include <boost/function.hpp>

class Guard
{
public:
    typedef boost::function<bool()> Condition;

public:
    Guard();
    Guard(Condition condition);

    bool evaluate() const;

private:
    Condition condition_;
};

#endif // GUARD_H
