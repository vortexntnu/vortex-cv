#include "vortex_yasmin_utils/persistent_waypoint_manager_state.hpp"

namespace vortex_yasmin_utils {

PersistentWaypointManagerState::PersistentWaypointManagerState(
    const std::string& action_server_name,
    double convergence_threshold)
    : ActionState(action_server_name,
                  std::bind(&PersistentWaypointManagerState::create_goal,
                            this,
                            std::placeholders::_1)),
      convergence_threshold_(convergence_threshold) {}

WaypointManagerAction::Goal PersistentWaypointManagerState::create_goal(
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    WaypointManagerAction::Goal goal;
    goal.waypoints = {};
    goal.persistent = true;
    goal.convergence_threshold = convergence_threshold_;

    return goal;
}

}  // namespace vortex_yasmin_utils
