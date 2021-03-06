/// HEADER
#include "lightfsm/meta_state.h"

MetaState::MetaState(State* parent)
  : State(parent), event_entry_meta(this, "initialize meta state"), event_exit_meta(this, "exit meta state")
{
}

void MetaState::entryAction()
{
    event_entry_meta.trigger();
}

std::vector<State*> MetaState::getChildren() const
{
    return children_;
}

void MetaState::registerChildState(State* child)
{
    children_.push_back(child);
}
