/// HEADER
#include "lightfsm/state_machine.h"

/// COMPONENT
#include "lightfsm/transition.h"
#include "lightfsm/meta_state.h"

/// SYSTEM
#include <fstream>
#include <iostream>
#include <sstream>

StateMachine::StateMachine(State* initial_state)
  : entry_state_(State::NO_PARENT)
  , start_state_(initial_state)
  , state_(&entry_state_)
  , state_change_pending_(true)
  , requested_state_(&entry_state_)
{
    entry_state_.event_default >> *initial_state;

    check();
}

void StateMachine::reset()
{
    state_change_pending_ = true;
    requested_state_ = start_state_;
}

void StateMachine::gotoState(State* state)
{
    state_change_pending_ = true;
    requested_state_ = state;
}

State* StateMachine::getState()
{
    return state_;
}

bool StateMachine::step()
{
    if (state_change_pending_) {
        state_ = requested_state_;
        state_change_pending_ = false;

        state_->performEntryAction();
    }

    std::vector<const Transition*> possible_transitions;

    state_->tick(possible_transitions);

    if (!possible_transitions.empty()) {
        // we can perform a transition
        if (possible_transitions.size() > 1) {
            std::cerr << possible_transitions.size() << " possible transitions, using the first one!" << std::endl;
        }
        perform(*possible_transitions.front());
    }

    return true;
}

void StateMachine::shutdown()
{
    state_->performExitAction();
}

template <class Stream>
Stream& StateMachine::printState(Stream& stream, const State* s, const std::string& prefix) const
{
    if (const MetaState* ms = dynamic_cast<const MetaState*>(s)) {
        //        stream << prefix << s->getName() << "_" << s->getUniqueId() << " [shape=record];\n";
        stream << "subgraph cluster_" << s->getUniqueId() << " {\n"
               << "node [style=filled, fontsize=10];\n"
               << "\"" << s->getName() << "_" << s->getUniqueId();
        if (s == state_) {
            stream << "\" [color=\"blue\",fontcolor=\"white\"];\n";
        } else {
            stream << "\" [color=\"white\",fontcolor=\"white\"];\n";
        }
        stream << "label = \"" << s->getName() << "\";\n";

        for (const State* nested : ms->getChildren()) {
            printState(stream, nested, prefix);
        }

        stream << "color=blue;\n"
               << "fontsize=24;\n"
               << "}\n";

    } else {
        stream << prefix << s->getName() << "_" << s->getUniqueId();
        stream << "[";
        if (s->getParent() == State::NO_PARENT) {
            stream << "fontsize=24";
        } else {
            stream << "fontsize=16";
        }
        if (s == state_) {
            stream << ";color=blue";
            stream << ";fontcolor=white";
            stream << ";style=filled";
        }
        stream << "]";

        stream << ";\n";
    }

    return stream;
}

template <class Stream>
Stream& StateMachine::printConnections(Stream& stream, const State* s, const std::string& prefix) const
{
    for (const Event* e : s->getEvents()) {
        std::vector<const Transition*> transitions;
        e->getAllTransitions(transitions);
        for (const Transition* t : transitions) {
            const State* target = t->getTarget();
            stream << prefix << s->getName() << "_" << s->getUniqueId() << " -> " << target->getName() << "_"
                   << target->getUniqueId() << "[label=\"" << t->getEvent()->getDescription() << "\"]"
                   << ";\n";
        }
    }
    return stream;
}

void StateMachine::check()
{
}

std::string StateMachine::generateGraphDescription() const
{
    std::stringstream graph;

    graph << "digraph fsm {\n";

    graph << "concentrate=true;\n";

    for (const State* s : State::g_states) {
        if (s->getParent() == State::NO_PARENT) {
            printState(graph, s, "  ");
        }
    }

    graph << "\n";

    for (const State* s : State::g_states) {
        printConnections(graph, s, "  ");
    }

    graph << "\n";

    graph << "{\n";
    graph << " rank=min;\n";
    graph << " " << start_state_->getName() << "_" << start_state_->getUniqueId() << ";\n";
    graph << "}\n";

    graph << "}";

    //    std::ofstream of("/tmp/graph.dot");
    //    of << graph.str();
    //    of.close();

    //    std::cerr << graph.str() << std::endl;
    //    std::abort();

    return graph.str();
}

void StateMachine::perform(const Transition& transition)
{
    State* next_state = transition.getTarget();

    std::cout << "// switching from state " << state_->getName() << " to " << next_state->getName() << '\n';

    transition.getEvent()->performTransitionActions();
    state_->performExitAction();
    transition.performAction();
    state_ = next_state;
    state_->performEntryAction();
}
