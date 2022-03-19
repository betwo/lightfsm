#ifndef EXPLORE_H
#define EXPLORE_H

/// COMPONENT
#include "../fsm/state.h"
#include "../fsm/triggered_event.h"
#include "../utils/map_explorer.h"

/// SYSTEM
#include <tf/tf.h>

class Explore : public State
{
public:
    TriggeredEvent event_object_found;

public:
    Explore(State* parent);

    void entryAction();
    void iteration();

    void exitAction();

protected:
    double desiredFrequency() const;

private:
    MapExplorer explorer_;
};

#endif  // EXPLORE_H
