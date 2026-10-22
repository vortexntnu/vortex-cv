#include "vortex_yasmin_utils/persistent_waypoint_manager_state.hpp"

namespace vortex_yasmin_utils {

PersistentWaypointManagerState::PersistentWaypointManagerState(
    const std::string& action_server_name)
    : ActionState(action_server_name,
                  std::bind(&PersistentWaypointManagerState::create_goal,
                            this,
                            std::placeholders::_1)) {}

WaypointManagerAction::Goal PersistentWaypointManagerState::create_goal(
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    WaypointManagerAction::Goal goal;
    goal.waypoints = {};
    goal.persistent = true;

    return goal;
}

}  // namespace vortex_yasmin_utils
