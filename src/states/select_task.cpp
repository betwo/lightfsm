#include "select_task.h"

#include "global_state.h"

SelectTask::SelectTask(State* parent)
    : State(parent),

      event_object_selected(this, "object selected"),
      event_all_objects_collected(this, "all objects collected"),
      event_object_unknown(this, "object unknown")
{

}

void SelectTask::entryAction()
{
    GlobalState& state = GlobalState::getInstance();

    if(state.isObjectCollected(sbc15_msgs::Object::OBJECT_BATTERY) &&
            state.isObjectCollected(sbc15_msgs::Object::OBJECT_CUP)) {
        event_all_objects_collected.trigger();
        return;
    }

    for(sbc15_msgs::Object& o : state.getObjects()) {
        if(!state.isObjectCollected(o.type)) {
            state.setCurrentObject(sbc15_msgs::ObjectPtr(new sbc15_msgs::Object(o)));
            event_object_selected.trigger();
            return;
        }
    }

    event_object_unknown.trigger();
}

void SelectTask::iteration()
{
    std::cerr << "i don't know what to do." << std::endl;
}

