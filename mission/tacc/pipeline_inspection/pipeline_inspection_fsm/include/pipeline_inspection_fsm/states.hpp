#ifndef PIPELINE_INSPECTION_FSM__STATES_HPP_
#define PIPELINE_INSPECTION_FSM__STATES_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/ros/ros_transforms.hpp>
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

#include <std_srvs/srv/trigger.hpp>
#include <vortex_msgs/action/landmark_polling.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/waypoint.hpp>

namespace pipeline_inspection_fsm {

using WaypointManagerAction = vortex_msgs::action::WaypointManager;
using WaypointManagerGoalHandle =
    rclcpp_action::ClientGoalHandle<WaypointManagerAction>;

using LandmarkPollingAction = vortex_msgs::action::LandmarkPolling;

using TriggerSrv = std_srvs::srv::Trigger;

}  // namespace pipeline_inspection_fsm

class TriggerWaitState : public yasmin::State {
   public:
    explicit TriggerWaitState(const std::string& service_name);

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
    void cancel_state() override;

   protected:
    virtual void on_triggered(yasmin::Blackboard::SharedPtr blackboard);

   private:
    void callback(pipeline_inspection_fsm::TriggerSrv::Request::SharedPtr req,
                  pipeline_inspection_fsm::TriggerSrv::Response::SharedPtr res);

    rclcpp::Service<pipeline_inspection_fsm::TriggerSrv>::SharedPtr service_;
    std::condition_variable cv_;
    std::mutex mutex_;
    bool triggered_{false};
};

class WaitForStartState : public TriggerWaitState {
   public:
    explicit WaitForStartState(yasmin::Blackboard::SharedPtr blackboard);
};

class SearchPatternState : public yasmin::State {
   public:
    explicit SearchPatternState(yasmin::Blackboard::SharedPtr blackboard);

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
    void cancel_state() override;

   private:
    std::vector<vortex::utils::types::Pose> load_search_waypoints() const;
    void cancel_active_goal();

    std::optional<pipeline_inspection_fsm::WaypointManagerAction::Goal>
    build_search_goal(const std::vector<vortex::utils::types::Pose>& waypoints);

    std::string waypoint_file_path_;
    std::string search_source_frame_;
    std::string search_target_frame_;
    double convergence_threshold_{};

    std::atomic<pipeline_inspection_fsm::WaypointManagerGoalHandle::SharedPtr>
        goal_handle_;

    rclcpp_action::Client<
        pipeline_inspection_fsm::WaypointManagerAction>::SharedPtr client_;
};

/*
 * Landmark polling action
 */
class LandmarkPollingState
    : public yasmin_ros::ActionState<
          pipeline_inspection_fsm::LandmarkPollingAction> {
   public:
    explicit LandmarkPollingState(yasmin::Blackboard::SharedPtr blackboard);

    pipeline_inspection_fsm::LandmarkPollingAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

    std::string result_handler(
        yasmin::Blackboard::SharedPtr blackboard,
        pipeline_inspection_fsm::LandmarkPollingAction::Result::SharedPtr
            result);

   private:
    int8_t landmark_type_{};
    int8_t landmark_subtype_{};
};

class ConvergeState : public yasmin_ros::ActionState<
                          pipeline_inspection_fsm::WaypointManagerAction> {
   public:
    explicit ConvergeState(yasmin::Blackboard::SharedPtr blackboard);

    pipeline_inspection_fsm::WaypointManagerAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    double convergence_threshold_{};
    vortex::utils::types::Pose pose_offset_{};
};

class StartWaypointManagerState : public yasmin::State {
   public:
    explicit StartWaypointManagerState(
        yasmin::Blackboard::SharedPtr blackboard);

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

   private:
    double convergence_threshold_{};

    rclcpp_action::Client<
        pipeline_inspection_fsm::WaypointManagerAction>::SharedPtr client_;
};

class WaitForPipelineEndState : public TriggerWaitState {
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
