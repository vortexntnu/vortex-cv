#include "pipeline_inspection_fsm/param_utils.hpp"
#include "pipeline_inspection_fsm/states.hpp"

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_clients_cache.hpp>
#include <yasmin_ros/yasmin_node.hpp>

using namespace std::chrono_literals;

StartWaypointManagerState::StartWaypointManagerState(
    yasmin::Blackboard::SharedPtr)
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::ABORT}) {
    auto node = yasmin_ros::YasminNode::get_instance();

    convergence_threshold_ = pipeline_inspection_fsm::param_utils::get_double(
        node, "fsm.convergence_threshold");

    client_ = yasmin_ros::ROSClientsCache::get_or_create_action_client<
        pipeline_inspection_fsm::WaypointManagerAction>(
        node, pipeline_inspection_fsm::param_utils::get_string(
                  node, "action_servers.waypoint_manager"));
}

std::string StartWaypointManagerState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
    if (!client_->wait_for_action_server(5s)) {
        YASMIN_LOG_ERROR(
            "StartWaypointManagerState: WaypointManager server not available");
        return yasmin_ros::basic_outcomes::ABORT;
    }

    pipeline_inspection_fsm::WaypointManagerAction::Goal goal;
    goal.persistent = true;
    goal.convergence_threshold = convergence_threshold_;

    auto future = client_->async_send_goal(goal);
    if (future.wait_for(5s) != std::future_status::ready) {
        YASMIN_LOG_ERROR(
            "StartWaypointManagerState: goal acceptance timed out");
        return yasmin_ros::basic_outcomes::ABORT;
    }

    auto wm_handle = future.get();
    if (!wm_handle) {
        YASMIN_LOG_ERROR("StartWaypointManagerState: goal rejected");
        return yasmin_ros::basic_outcomes::ABORT;
    }

    blackboard
        ->set<pipeline_inspection_fsm::WaypointManagerGoalHandle::SharedPtr>(
            "wm_handle", wm_handle);

    YASMIN_LOG_INFO(
        "StartWaypointManagerState: WaypointManager running in persistent "
        "mode");

    return yasmin_ros::basic_outcomes::SUCCEED;
}
