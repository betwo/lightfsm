#ifndef sbc15_GLOBAL_H
#define sbc15_GLOBAL_H

/// COMPONENT
#include "states/global_state.h"
#include "fsm/state.h"

namespace sbc15_fsm_global
{
namespace action
{
void print(const std::string& str);
void say(const std::string& str);
}  // namespace action

void waitForRosTime();

}  // namespace sbc15_fsm_global

class Initial : public State
{
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

#endif  // sbc15_GLOBAL_H
