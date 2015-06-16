/// HEADER
#include "meta_state.h"

MetaState::MetaState(State* parent)
    : State(parent),
      event_entry_meta(this, "initialize meta state"),
      event_exit_meta(this, "exit meta state")
{
}

void MetaState::entryAction()
{
    event_entry_meta.trigger();
}
