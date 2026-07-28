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
    // TODO(member) 1 of 3 — construct your HelloState.
    //
    //   auto hello = std::make_shared<vortex_yasmin_utils::HelloState>();
    //
    // (Its execute() body still has its own TODO — see
    //  mission/vortex_yasmin_utils/src/hello_state.cpp. It'll compile and
    //  run either way.)
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // TODO(member) 2 of 3 — construct one WaypointGoalState per waypoint.
    //
    // initialize_blackboard() (see blackboard.cpp) already loaded all 4
    // WaypointGoal structs from config/waypoints.yaml onto `blackboard`
    // under the keys "waypoint_1_goal" .. "waypoint_4_goal". Pull each one
    // off and hand it to a vortex_yasmin_utils::WaypointGoalState:
    //
    //   auto wp1 = std::make_shared<vortex_yasmin_utils::WaypointGoalState>(
    //       config.waypoint_manager_action_server,
    //       blackboard->get<vortex::utils::waypoints::WaypointGoal>(
    //           "waypoint_1_goal"));
    //
    // Repeat for "waypoint_2_goal" .. "waypoint_4_goal" (wp2, wp3, wp4).
    //
    // For a real example of pulling a WaypointGoal off the blackboard and
    // handing it straight to an action state, see:
    //   mission/tacc/subsea_docking/subsea_docking_fsm/src/build_state_machine.cpp
    //   (search for "dock_config_waypoint_goal").
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // TODO(member) 3 of 3 — wire the transition table.
    //
    // Target machine:
    //   HELLO -> WP1 -> WP2 -> WP3 -> WP4 -> DONE
    //
    // Every state above only ever produces SUCCEED, ABORT, or CANCEL. Add
    // each one with sm->add_state(name, state, {outcome -> next_state, ...}).
    // On ABORT or CANCEL, send the machine straight to the terminal ABORT
    // outcome — don't try to recover mid-sequence. For example:
    //
    //   sm->add_state("HELLO", hello,
    //                 {{SUCCEED, "WP1"}, {ABORT, ABORT}});
    //   sm->add_state("WP1", wp1,
    //                 {{SUCCEED, "WP2"}, {ABORT, ABORT}, {CANCEL, ABORT}});
    //   sm->add_state("WP2", wp2,
    //                 {{SUCCEED, "WP3"}, {ABORT, ABORT}, {CANCEL, ABORT}});
    //   sm->add_state("WP3", wp3,
    //                 {{SUCCEED, "WP4"}, {ABORT, ABORT}, {CANCEL, ABORT}});
    //   sm->add_state("WP4", wp4,
    //                 {{SUCCEED, "DONE"}, {ABORT, ABORT}, {CANCEL, ABORT}});
    //   sm->add_state(
    //       "DONE",
    //       yasmin::CbState::make_shared(
    //           yasmin::Outcomes{SUCCEED},
    //           [](yasmin::Blackboard::SharedPtr) -> std::string {
    //               YASMIN_LOG_INFO("Learning period mission complete");
    //               return SUCCEED;
    //           }),
    //       {{SUCCEED, SUCCEED}});
    //
    // The FIRST state you add_state() becomes the machine's entry point, so
    // "HELLO" must be the first add_state() call you make.
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
