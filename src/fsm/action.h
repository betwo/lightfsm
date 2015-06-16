#ifndef ACTION_H
#define ACTION_H

/// SYSTEM
#include <boost/function.hpp>
#include <vector>

class Action
{
public:
    typedef boost::function<void()> Callable;

public:
    Action(Callable call);
    Action();

    void perform() const;

private:
    Callable call_;
};




inline void operator << (std::vector<Action>& v, const Action& a)
{
    v.push_back(a);
}
inline void operator << (std::vector<Action>& v, const boost::function<void()>& a)
{
    v.push_back(Action(a));
}

#endif // ACTION_H
