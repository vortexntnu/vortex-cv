#include "vortex_bt_nodes/temp/register.hpp"

#include <spdlog/spdlog.h>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/measure_landmark.hpp>

#include "vortex_bt_nodes/map/landmark_map.hpp"

namespace vortex_bt_nodes::temp {

namespace {

/// Calls landmark_server/measure. SUCCESS when measured, or when the
/// fallback was copied and accept_fallback is true.
class Measure : public BT::StatefulActionNode {
   public:
    using Action = vortex_msgs::action::MeasureLandmark;
    using Handle = rclcpp_action::ClientGoalHandle<Action>;

    Measure(const std::string& name,
            const BT::NodeConfig& config,
            rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(std::move(node)) {
        client_ = rclcpp_action::create_client<Action>(
            node_, "landmark_server/measure");
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("class_name"),
                BT::InputPort<std::string>("out_frame", "", ""),
                BT::InputPort<double>("duration_s", 0.0, "0 = server default"),
                BT::InputPort<std::string>("fallback_from", "", ""),
                BT::InputPort<bool>("accept_fallback", true, "")};
    }

   private:
    BT::NodeStatus onStart() override {
        const auto cls = getInput<std::string>("class_name");
        if (!cls || !client_->action_server_is_ready()) {
            spdlog::error("[{}] no class_name or no measure server", name());
            return BT::NodeStatus::FAILURE;
        }
        Action::Goal goal;
        goal.class_name = *cls;
        goal.out_frame = getInput<std::string>("out_frame").value_or("");
        goal.duration_s = getInput<double>("duration_s").value_or(0.0);
        goal.fallback_from = getInput<std::string>("fallback_from").value_or("");
        result_.reset();
        handle_.reset();
        auto options = rclcpp_action::Client<Action>::SendGoalOptions();
        options.goal_response_callback = [this](Handle::SharedPtr h) {
            handle_ = h;
            if (!h) {
                rejected_ = true;
            }
        };
        options.result_callback = [this](const Handle::WrappedResult& r) {
            result_ = r;
        };
        rejected_ = false;
        client_->async_send_goal(goal, options);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (rejected_) {
            spdlog::warn("[{}] measure rejected", name());
            return BT::NodeStatus::FAILURE;
        }
        if (!result_) {
            return BT::NodeStatus::RUNNING;
        }
        const auto& r = *result_->result;
        spdlog::info("[{}] {}: {}{}", name(),
                     getInput<std::string>("class_name").value_or(""),
                     r.success ? "measured, " : "failed, ", r.message);
        const bool fallback_ok =
            r.used_fallback && getInput<bool>("accept_fallback").value_or(true);
        return r.success || fallback_ok ? BT::NodeStatus::SUCCESS
                                        : BT::NodeStatus::FAILURE;
    }

    void onHalted() override {
        if (handle_) {
            client_->async_cancel_goal(handle_);
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<Action>::SharedPtr client_;
    Handle::SharedPtr handle_;
    std::optional<Handle::WrappedResult> result_;
    bool rejected_{false};
};

/// SUCCESS when base_link is within xy_m and yaw_deg of frame + offset.
class WithinThreshold : public BT::ConditionNode {
   public:
    WithinThreshold(const std::string& name,
                    const BT::NodeConfig& config,
                    std::shared_ptr<const map::LandmarkMap> map)
        : BT::ConditionNode(name, config), map_(std::move(map)) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("frame"),
                BT::InputPort<Pose>("offset", "0;0;0", ""),
                BT::InputPort<double>("xy_m", 0.15, ""),
                BT::InputPort<double>("yaw_deg", 10.0, "")};
    }

   private:
    BT::NodeStatus tick() override {
        const auto frame = getInput<std::string>("frame");
        const auto offset = getInput<Pose>("offset");
        if (!frame || !offset) {
            return BT::NodeStatus::FAILURE;
        }
        const auto target_frame =
            map_->lookup(map_->odom_frame(), map_->frame_name(*frame));
        const auto vehicle = map_->vehicle_in_odom();
        if (!target_frame || !vehicle) {
            return BT::NodeStatus::FAILURE;
        }
        const Pose target = map::LandmarkMap::compose(*target_frame, *offset);
        const double dxy = std::hypot(target.position.x - vehicle->position.x,
                                      target.position.y - vehicle->position.y);
        tf2::Quaternion a, b;
        tf2::fromMsg(target.orientation, a);
        tf2::fromMsg(vehicle->orientation, b);
        const double dyaw = std::abs(std::remainder(
                                tf2::getYaw(a) - tf2::getYaw(b), 2.0 * M_PI)) *
                            180.0 / M_PI;
        const bool ok = dxy <= getInput<double>("xy_m").value_or(0.15) &&
                        dyaw <= getInput<double>("yaw_deg").value_or(10.0);
        spdlog::info("[{}] {}: {:.2f} m, {:.1f} deg from it -> {}", name(),
                     *frame, dxy, dyaw, ok ? "within" : "not within");
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    std::shared_ptr<const map::LandmarkMap> map_;
};

}  // namespace

void register_nodes(BT::BehaviorTreeFactory& factory,
                    const rclcpp::Node::SharedPtr& node) {
    factory.registerNodeType<Measure>("Measure", node);
    factory.registerNodeType<WithinThreshold>(
        "WithinThreshold", std::make_shared<map::LandmarkMap>(node));
}

}  // namespace vortex_bt_nodes::temp
