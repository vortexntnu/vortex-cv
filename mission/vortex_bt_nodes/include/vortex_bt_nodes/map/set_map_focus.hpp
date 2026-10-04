#ifndef VORTEX_BT_NODES__MAP__SET_MAP_FOCUS_HPP_
#define VORTEX_BT_NODES__MAP__SET_MAP_FOCUS_HPP_

#include <behaviortree_cpp/action_node.h>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <vortex_msgs/srv/set_map_focus.hpp>

namespace vortex_bt_nodes::map {

/// "slalom_1, slalom_2" -> {"slalom_1", "slalom_2"}; empty -> {}.
std::vector<std::string> split_task_list(const std::string& text);

/**
 * @brief Tell landmark_server which tasks the mission works on now
 * (landmark_server/set_focus). With lock_others the other tasks are frozen:
 * their parts are not added or moved (only the drift correction moves them),
 * and detections of their classes outside the focused tasks are dropped.
 * commit freezes a task's pose and variant (e.g. once the torpedo board has
 * been seen well); uncommit releases it.
 *
 * Ports (task lists comma-separated):
 *  - tasks: the tasks in focus; empty = all (search mode)
 *  - lock_others (default true)
 *  - commit, uncommit: tasks to freeze / release
 *
 * SUCCESS when the server accepts, FAILURE when it refuses (unknown task,
 * no course layout) or is not there within service_timeout_s. Never blocks
 * a tick.
 *
 * Example:
 * @code
 * <SetMapFocus tasks="torpedo" lock_others="true"/>
 * <SetMapFocus tasks="" commit="torpedo"/>
 * @endcode
 */
class SetMapFocus : public BT::StatefulActionNode {
   public:
    using Srv = vortex_msgs::srv::SetMapFocus;

    SetMapFocus(const std::string& name,
                const BT::NodeConfig& config,
                rclcpp::Node::SharedPtr node,
                const std::string& service_name = "landmark_server/set_focus");

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<Srv>::SharedPtr client_;
    std::optional<rclcpp::Client<Srv>::SharedFuture> future_;
    std::optional<int64_t> request_id_;
    rclcpp::Time deadline_;
    std::shared_ptr<Srv::Request> request_;
};

}  // namespace vortex_bt_nodes::map

#endif  // VORTEX_BT_NODES__MAP__SET_MAP_FOCUS_HPP_
