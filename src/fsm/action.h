#ifndef ACTION_H
#define ACTION_H

/// SYSTEM
#include <functional>
#include <vector>

class Action
{
public:
    typedef std::function<void()> Callable;

public:
    Action(Callable call);
    Action();

    void perform() const;

private:
    Callable call_;
};



#endif // ACTION_H
