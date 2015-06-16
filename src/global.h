#ifndef SICK14_GLOBAL_H
#define SICK14_GLOBAL_H

/// COMPONENT
#include "states/global_state.h"
#include "fsm/state.h"

namespace sick14_fsm_global {
namespace action {
void print(const std::string& str);
void say(const std::string& str);
}

void waitForRosTime();

}


class Initial : public State {
public:
    Initial(State* parent);
};

class Error : public State
{
public:
    Error(State* parent);

protected:
    bool isTerminal() const;
};

class Quit : public State
{
public:
    Quit(State* parent);

protected:
    bool isTerminal() const;
};

#endif // SICK14_GLOBAL_H
