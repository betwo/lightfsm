/// HEADER
#include "fetch_object.h"

/// COMPONENT
#include "../states/global_state.h"


FetchObject::FetchObject(State* parent, bool store)
    : MetaState(parent),

      event_object_unknown(this, "The object is not known"),
      event_object_fetched(this, "The object has been fetched"),
      event_failure(this, "An error happened"),

      goto_object(this, 1.2 /*m*/),
      approach(this, 0.55, 0.1),
      pickup_object(this, store)
{
    event_entry_meta >> goto_object;

    goto_object.event_object_reached >> approach;
    goto_object.event_path_failure >> goto_object;
    goto_object.event_object_unknown >> event_object_unknown;

    approach.event_approached >> pickup_object;
    approach.event_failure >> goto_object;
    approach.event_orientation_mismatch >> goto_object;

    pickup_object.event_object_pickedup >> event_object_fetched;
    pickup_object.event_object_failure >> event_failure;
    pickup_object.event_object_out_of_range >> goto_object;
    pickup_object.event_servo_control_failed >> event_failure;
    pickup_object.event_planning_failed >> event_failure;
}
