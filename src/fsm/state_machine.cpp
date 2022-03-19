/// HEADER
#include "state_machine.h"

/// COMPONENT
#include "transition.h"
#include "meta_state.h"
#include "../global.h"

/// SYSTEM
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <std_msgs/String.h>

StateMachine::StateMachine(State* initial_state)
  : start_state_(initial_state), state_(initial_state), reset_(false), reset_state_(nullptr)
{
    check();

    state_->performEntryAction();
}

namespace
{
void kill_sub(const std_msgs::BoolConstPtr& /*kill*/, bool& k)
{
    k = true;
}
}  // namespace

void StateMachine::reset()
{
    reset_ = true;
    reset_state_ = start_state_;
}

void StateMachine::gotoState(State* state)
{
    reset_ = true;
    reset_state_ = state;
}

void StateMachine::run(std::function<void(State*)> callback)
{
    ROS_INFO_STREAM("starting with state " << state_->getName());

    bool kill = false;

    ros::NodeHandle pnh("~");
    ros::Subscriber sub = pnh.subscribe<std_msgs::Bool>("kill", 1, boost::bind(&kill_sub, _1, kill));

    while (ros::ok()) {
        callback(state_);
        bool terminal = !step();

        if (terminal) {
            return;
        }

        if (kill) {
            return;
        }

        if (reset_) {
            sbc15_fsm_global::action::say("resetting");

            state_ = reset_state_;
            state_->performEntryAction();

            ROS_WARN_STREAM("resetting state machine, going to state " << state_->getName());
            reset_ = false;
        }

        // handle ros stuff
        ros::spinOnce();
        state_->getRate().sleep();
    }
}

bool StateMachine::step()
{
    std::vector<const Transition*> possible_transitions;

    state_->tick(possible_transitions);

    // can we perform a transition?
    if (!possible_transitions.empty()) {
        if (possible_transitions.size() > 1) {
            std::cerr << possible_transitions.size() << " possible transitions, using the first one!" << std::endl;
        }
        perform(*possible_transitions.front());

    } else if (state_->isTerminal()) {
        state_->performExitAction();
        return false;
    }

    return true;
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

    ROS_INFO_STREAM("switching from state " << state_->getName() << " to " << next_state->getName());

    state_->performExitAction();
    transition.getEvent()->performTransitionActions();
    transition.performAction();
    state_ = next_state;
    state_->performEntryAction();
}
