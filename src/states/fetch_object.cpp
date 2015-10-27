/// HEADER
#include "fetch_object.h"

/// COMPONENT
#include "../states/global_state.h"


FetchObject::FetchObject(State* parent, bool store,ArmGoal& goal )
    : MetaState(parent),

      event_object_unknown(this, "The object is not known"),

      goto_object(this),
      pickup_object(this, store, goal)
{
    event_entry_meta >> goto_object;
    goto_object.event_object_reached >> pickup_object;

    goto_object.event_path_failure >> goto_object; // TODO
    goto_object.event_object_unknown >> event_object_unknown;
}
