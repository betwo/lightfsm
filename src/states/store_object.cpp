#include "store_object.h"

StoreObject::StoreObject(State *parent):
    State(parent),
    object_stored(this,"object is stored"),
    event_failure(this,"error occured")

{
}
