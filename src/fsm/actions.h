#ifndef ACTIONS_H
#define ACTIONS_H

/// COMPONENT
#include "action.h"

/// SYSTEM
#include <vector>

class Event;
class Transition;

class Actions
{
public:
    Actions() = default;

    void push_back(const Action& action);

    std::vector<Action>::iterator begin();

    std::vector<Action>::const_iterator begin() const;

    std::vector<Action>::iterator end();

    std::vector<Action>::const_iterator end() const;

    Actions& operator << (const Action& a);

    Actions& operator << (const std::function<void()>& a);

private:
    std::vector<Action> actions_;
};


#endif /* ACTIONS_H */
