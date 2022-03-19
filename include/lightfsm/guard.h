#ifndef GUARD_H
#define GUARD_H

/// SYSTEM
#include <functional>

class Guard
{
public:
    typedef std::function<bool()> Condition;

public:
    Guard();
    Guard(Condition condition);

    bool evaluate() const;

private:
    Condition condition_;
};

#endif  // GUARD_H
