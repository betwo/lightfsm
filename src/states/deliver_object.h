#ifndef DELIVER_OBJECT_H
#define DELIVER_OBJECT_H

/// COMPONENT
#include "../fsm/meta_state.h"
#include "../fsm/triggered_event.h"
#include "../states/go_to_base.h"
#include "../states/place_cup.h"
#include "ros/ros.h"

class DeliverObject : public MetaState
{
public:
    TriggeredEvent event_done;
    TriggeredEvent event_failure;

public:
    GoToBase got_to_base;
    PlaceCup place_cup;

public:
    DeliverObject(State* parent);
};

#endif  // DELIVER_OBJECT_H
