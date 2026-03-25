#ifndef PIPELINE_INSPECTION_FSM__STATES_HPP_
#define PIPELINE_INSPECTION_FSM__STATES_HPP_

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/types.hpp>
#include <vortex/utils/waypoint_utils.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin/logs.hpp>
#include <yasmin/state.hpp>

#include <yasmin_ros/action_state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_clients_cache.hpp>
#include <yasmin_ros/service_state.hpp>
#include <yasmin_ros/yasmin_node.hpp>

#include <vortex_yasmin_utils/landmark_converge_state.hpp>
#include <vortex_yasmin_utils/landmark_polling_state.hpp>
#include <vortex_yasmin_utils/trigger_wait_state.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include <vortex_msgs/msg/waypoint.hpp>

namespace pipeline_inspection_fsm {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;
using WaypointManagerGoalHandle =
    rclcpp_action::ClientGoalHandle<WaypointManagerAction>;

using TriggerSrv = std_srvs::srv::Trigger;

}  // namespace pipeline_inspection_fsm

class WaitForStartState : public vortex_yasmin_utils::TriggerWaitState {
   public:
    explicit WaitForStartState(yasmin::Blackboard::SharedPtr blackboard);
};

class SearchPatternState : public yasmin::State {
   public:
    explicit SearchPatternState(yasmin::Blackboard::SharedPtr blackboard);

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
    void cancel_state() override;

   private:
    std::vector<vortex::utils::waypoints::WaypointGoal> load_search_waypoints()
        const;
    void cancel_active_goal();

    std::optional<pipeline_inspection_fsm::WaypointManagerAction::Goal>
    build_search_goal(
        const std::vector<vortex::utils::waypoints::WaypointGoal>& waypoints);

    std::string waypoint_file_path_;

    std::atomic<pipeline_inspection_fsm::WaypointManagerGoalHandle::SharedPtr>
        goal_handle_;

    rclcpp_action::Client<
        pipeline_inspection_fsm::WaypointManagerAction>::SharedPtr client_;
};

class StartWaypointManagerState : public yasmin::State {
   public:
    explicit StartWaypointManagerState(
        yasmin::Blackboard::SharedPtr blackboard);

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

   private:
    rclcpp_action::Client<
        pipeline_inspection_fsm::WaypointManagerAction>::SharedPtr client_;
};

class WaitForPipelineEndState : public vortex_yasmin_utils::TriggerWaitState {
   public:
    explicit WaitForPipelineEndState(yasmin::Blackboard::SharedPtr blackboard);

   protected:
    void on_triggered(yasmin::Blackboard::SharedPtr blackboard) override;
};

class StopWaypointManagerState : public yasmin::State {
   public:
    explicit StopWaypointManagerState(yasmin::Blackboard::SharedPtr blackboard);

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

   private:
    rclcpp_action::Client<
        pipeline_inspection_fsm::WaypointManagerAction>::SharedPtr client_;
};

std::shared_ptr<yasmin::Blackboard> initialize_blackboard();

#endif  // PIPELINE_INSPECTION_FSM__STATES_HPP_
