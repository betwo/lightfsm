#include "deliver_object.h"

DeliverObject::DeliverObject(State* parent)
  : MetaState(parent)
  , event_done(this, "Cup Delivered")
  , event_failure(this, "Failure")
  , got_to_base(this)
  , place_cup(this)
{
    event_entry_meta >> got_to_base;
    got_to_base.event_done >> place_cup;
    place_cup.event_cup_placed >> event_done;

    place_cup.event_failure >> event_failure;
}
