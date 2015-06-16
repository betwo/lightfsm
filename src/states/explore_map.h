#ifndef EXPLORE_MAP_H
#define EXPLORE_MAP_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
#include "../utils/map_explorer.h"

/// SYSTEM
#include <tf/tf.h>

class ExploreMap : public State
{
public:
    TriggeredEvent event_step;

public:
    ExploreMap(State* parent);

    void entryAction();
    void iteration();
    void exitAction();

private:
    MapExplorer explorer_;
};

#endif // EXPLORE_MAP_H
