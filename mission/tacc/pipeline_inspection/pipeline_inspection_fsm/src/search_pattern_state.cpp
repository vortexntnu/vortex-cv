#include "pipeline_inspection_fsm/states.hpp"

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_clients_cache.hpp>
#include <yasmin_ros/yasmin_node.hpp>

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

using namespace std::chrono_literals;

SearchPatternState::SearchPatternState(yasmin::Blackboard::SharedPtr blackboard)
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::ABORT,
                     yasmin_ros::basic_outcomes::CANCEL}),
      waypoint_file_path_(blackboard->get<std::string>("waypoint_file_path")) {
    auto node = yasmin_ros::YasminNode::get_instance();

    client_ = yasmin_ros::ROSClientsCache::get_or_create_action_client<
        pipeline_inspection_fsm::WaypointManagerAction>(
        node, blackboard->get<std::string>("action_server.waypoint_manager"));
}

std::vector<vortex::utils::waypoints::WaypointGoal>
SearchPatternState::load_search_waypoints() const {
    std::vector<vortex::utils::waypoints::WaypointGoal> waypoints;

    for (int i = 1;; ++i) {
        const std::string id = "search_waypoint_" + std::to_string(i);
        try {
            waypoints.push_back(
                vortex::utils::waypoints::load_waypoint_goal_from_yaml(
                    waypoint_file_path_, id));
        } catch (const std::runtime_error&) {
            break;
        }
    }

    return waypoints;
}

void SearchPatternState::cancel_active_goal() {
    if (auto h = goal_handle_.load())
        client_->async_cancel_goal(h);
}

std::optional<pipeline_inspection_fsm::WaypointManagerAction::Goal>
SearchPatternState::build_search_goal(
    const std::vector<vortex::utils::waypoints::WaypointGoal>& waypoints) {
    std::vector<vortex_msgs::msg::Waypoint> waypoint_msgs;
    waypoint_msgs.reserve(waypoints.size());

    for (const auto& wp : waypoints) {
        vortex_msgs::msg::Waypoint waypoint_msg;
        waypoint_msg.pose =
            vortex::utils::ros_conversions::to_pose_msg(wp.pose);
        waypoint_msg.waypoint_mode.mode = static_cast<uint8_t>(wp.mode);
        waypoint_msgs.push_back(waypoint_msg);
    }

    pipeline_inspection_fsm::WaypointManagerAction::Goal goal;
    goal.waypoints = waypoint_msgs;
    goal.persistent = false;
    goal.convergence_threshold = waypoints.front().convergence_threshold;

    return goal;
}

std::string SearchPatternState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
    if (!client_->wait_for_action_server(5s)) {
        YASMIN_LOG_ERROR(
            "SearchPatternState: WaypointManager server not available");
        return yasmin_ros::basic_outcomes::ABORT;
    }

    const auto waypoints = load_search_waypoints();
    if (waypoints.empty()) {
        YASMIN_LOG_ERROR("SearchPatternState: no search waypoints configured");
        return yasmin_ros::basic_outcomes::ABORT;
    }

    blackboard->set<bool>("landmark_found", false);

    const auto goal = build_search_goal(waypoints);
    if (!goal)
        return yasmin_ros::basic_outcomes::ABORT;

    YASMIN_LOG_INFO("SearchPatternState: sending %zu waypoints",
                    waypoints.size());

    auto future = client_->async_send_goal(*goal);

    if (future.wait_for(5s) != std::future_status::ready) {
        YASMIN_LOG_ERROR("SearchPatternState: goal acceptance timed out");
        return yasmin_ros::basic_outcomes::ABORT;
    }

    auto handle = future.get();
    if (!handle) {
        YASMIN_LOG_ERROR("SearchPatternState: goal rejected");
        return yasmin_ros::basic_outcomes::ABORT;
    }

    goal_handle_.store(handle);

    auto result_future = client_->async_get_result(handle);

    while (result_future.wait_for(100ms) != std::future_status::ready) {
        if (is_canceled()) {
            cancel_active_goal();
            return yasmin_ros::basic_outcomes::CANCEL;
        }
        if (blackboard->get<bool>("landmark_found")) {
            cancel_active_goal();
            return yasmin_ros::basic_outcomes::SUCCEED;
        }
    }

    goal_handle_.store(nullptr);

    YASMIN_LOG_WARN(
        "SearchPatternState: exhausted search waypoints without finding "
        "landmark");
    return yasmin_ros::basic_outcomes::ABORT;
}

void SearchPatternState::cancel_state() {
    cancel_active_goal();
    yasmin::State::cancel_state();
}
