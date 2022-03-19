/// HEADER
#include "actions.h"

void Actions::push_back(const Action& action)
{
    actions_.push_back(action);
}

std::vector<Action>::iterator Actions::begin()
{
    return actions_.begin();
}

std::vector<Action>::const_iterator Actions::begin() const
{
    return actions_.begin();
}

std::vector<Action>::iterator Actions::end()
{
    return actions_.end();
}

std::vector<Action>::const_iterator Actions::end() const
{
    return actions_.end();
}

Actions& Actions::operator<<(const Action& a)
{
    actions_.push_back(a);
    return *this;
}
Actions& Actions::operator<<(const std::function<void()>& a)
{
    actions_.emplace_back(a);
    return *this;
}