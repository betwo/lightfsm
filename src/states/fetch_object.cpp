/// HEADER
#include "fetch_object.h"

/// COMPONENT
#include "../states/global_state.h"


FetchObject::FetchObject(State* parent, bool store)
    : MetaState(parent),

      event_object_unknown(this, "The object is not known"),
      event_object_fetched(this, "The object has been fetched"),
      event_failure(this, "An error happened"),

      goto_before_object(this, 0.9 /*m*/),
      wait(this, 10),
      goto_object(this, 0.65 /*m*/),
      pickup_object(this, store)
{
    event_entry_meta >> goto_before_object;

    goto_before_object.event_object_reached >> wait;
    goto_before_object.event_path_failure >> goto_object; // TODO
    goto_before_object.event_object_unknown >> event_object_unknown;

    wait.event_done >> goto_object;

    goto_object.event_object_reached >> pickup_object;
    goto_object.event_path_failure >> goto_object; // TODO
    goto_object.event_object_unknown >> event_object_unknown;

    pickup_object.event_object_pickedup >> event_object_fetched;
    pickup_object.event_object_failure >> event_failure;
    pickup_object.event_object_out_of_range >> goto_before_object;
    pickup_object.event_servo_control_failed >> event_failure;
    pickup_object.event_planning_failed >> event_failure;
}
