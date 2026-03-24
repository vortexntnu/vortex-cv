#include "pipeline_inspection_fsm/param_utils.hpp"
#include "pipeline_inspection_fsm/states.hpp"

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_clients_cache.hpp>
#include <yasmin_ros/yasmin_node.hpp>

StopWaypointManagerState::StopWaypointManagerState(
    yasmin::Blackboard::SharedPtr)
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}) {
    auto node = yasmin_ros::YasminNode::get_instance();

    client_ = yasmin_ros::ROSClientsCache::get_or_create_action_client<
        pipeline_inspection_fsm::WaypointManagerAction>(
        node, pipeline_inspection_fsm::param_utils::get_string(
                  node, "action_servers.waypoint_manager"));
}

std::string StopWaypointManagerState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
    auto handle = blackboard->get<
        pipeline_inspection_fsm::WaypointManagerGoalHandle::SharedPtr>(
        "wm_handle");

    if (!handle) {
        YASMIN_LOG_WARN("StopWaypointManagerState: no active handle found");
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    YASMIN_LOG_INFO("StopWaypointManagerState: cancelling WaypointManager");
    client_->async_cancel_goal(handle);

    blackboard
        ->set<pipeline_inspection_fsm::WaypointManagerGoalHandle::SharedPtr>(
            "wm_handle", nullptr);

    return yasmin_ros::basic_outcomes::SUCCEED;
}
