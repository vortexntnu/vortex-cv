#include "learning_period/states.hpp"

#include <set>

#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_yasmin_utils/hello_state.hpp>
#include <vortex_yasmin_utils/waypoint_goal_state.hpp>

#include <yasmin/cb_state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

using yasmin_ros::basic_outcomes::ABORT;
using yasmin_ros::basic_outcomes::CANCEL;
using yasmin_ros::basic_outcomes::SUCCEED;

std::shared_ptr<yasmin::StateMachine> build_state_machine(
    const StateMachineConfig& config,
    yasmin::Blackboard::SharedPtr blackboard) {
    (void)config;
    (void)blackboard;

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{SUCCEED, ABORT});

    // ------------------------------------------------------------------
    // Reference implementation for everything below:
    //   mission/tacc/visual_inspection/visual_inspection_fsm/src/
    //       build_state_machine.cpp
    // It is a real mission FSM built exactly the same way — read it first.
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // TODO(member) 1 of 3 — construct your HelloState.
    //
    // It takes no constructor arguments. See how the reference file
    // constructs WipeState (also argument-less) for the pattern.
    // Implement its execute() body too:
    //   mission/vortex_yasmin_utils/src/hello_state.cpp
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // TODO(member) 2 of 3 — construct one WaypointGoalState per waypoint.
    //
    // WaypointGoalState's constructor (see its header) takes:
    //   1. the action server name — on `config`, filled in from a ROS
    //      parameter in load_config(), see blackboard.cpp
    //   2. a vortex::utils::waypoints::WaypointGoal
    //
    // Nothing here hardcodes poses. initialize_blackboard() (blackboard.cpp)
    // read all 4 goals out of config/waypoints.yaml and put them on
    // `blackboard` under the keys "waypoint_1_goal" .. "waypoint_4_goal".
    // Pull each one back off with
    // blackboard->get<vortex::utils::waypoints::WaypointGoal>(<key>) and
    // hand it to the state.
    //
    // The reference file does this at the top of build_state_machine() with
    // "standoff_goal" / "tcp_offset_goal" — same call, different keys.
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // TODO(member) 3 of 3 — wire the transition table.
    //
    // Target machine:
    //   HELLO -> WP1 -> WP2 -> WP3 -> WP4 -> DONE
    //
    // Each state is registered with
    //   sm->add_state(<name>, <state>, {{<outcome>, <next>}, ...});
    // where <name> is a label you choose, and the third argument maps every
    // outcome that state can return to the name of the next state. An
    // outcome may also map to one of the machine's terminal outcomes
    // (SUCCEED / ABORT, the set passed to the StateMachine constructor
    // above) — that ends the machine instead of moving to another state.
    //
    // Rules that bite people:
    //   - The FIRST add_state() call is the machine's entry point, so HELLO
    //     must come first.
    //   - Every outcome a state declares must appear in its map, or yasmin
    //     throws at construction time. HelloState declares only SUCCEED;
    //     WaypointGoalState (an ActionState) declares SUCCEED, ABORT and
    //     CANCEL.
    //   - Names in the map are resolved when the machine is built, so a
    //     typo'd target name fails loudly at startup, not mid-mission.
    //
    // For this exercise, send ABORT and CANCEL straight to the terminal
    // ABORT outcome — no retrying mid-sequence (stretch goal 3 in the README
    // changes that).
    //
    // The reference file shows both halves of this: real states chained by
    // name, and a final "DONE" state built inline with
    // yasmin::CbState::make_shared(...) that just logs and returns SUCCEED.
    // Build your DONE the same way.
    // ------------------------------------------------------------------

    // Placeholder so the scaffold compiles and runs cleanly before the TODOs
    // above are filled in. Delete this once you've added your own states.
    sm->add_state(
        "PLACEHOLDER",
        yasmin::CbState::make_shared(
            yasmin::Outcomes{SUCCEED},
            [](yasmin::Blackboard::SharedPtr) -> std::string {
                YASMIN_LOG_WARN(
                    "learning_period_fsm: no states wired yet, see TODOs in "
                    "build_state_machine.cpp");
                return SUCCEED;
            }),
        {{SUCCEED, SUCCEED}});

    return sm;
}
