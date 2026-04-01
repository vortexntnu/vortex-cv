#ifndef VALVE_INSPECTION_FSM__VALVE_CONVERGE_STATE_HPP_
#define VALVE_INSPECTION_FSM__VALVE_CONVERGE_STATE_HPP_

#include <memory>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vortex_msgs/action/landmark_convergence.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin_ros/action_state.hpp>

namespace valve_inspection_fsm {

using LandmarkConvergenceAction = vortex_msgs::action::LandmarkConvergence;

/**
 * @brief Valve-specific convergence state using the LandmarkConvergence action.
 *
 * Looks up the base_frame -> gripper_frame transform via TF2 at goal creation
 */
class ValveConvergeState
    : public yasmin_ros::ActionState<LandmarkConvergenceAction> {
   public:
    ValveConvergeState(const std::string& action_server_name,
                       vortex_msgs::msg::LandmarkType type,
                       vortex_msgs::msg::LandmarkSubtype subtype,
                       double convergence_threshold = 0.05,
                       double dead_reckoning_threshold = 0.5,
                       double track_loss_timeout_sec = 5.0,
                       const std::string& gripper_frame = "gripper_tip",
                       const std::string& base_frame = "base_link");

    LandmarkConvergenceAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

   private:
    vortex_msgs::msg::LandmarkType type_;
    vortex_msgs::msg::LandmarkSubtype subtype_;
    double convergence_threshold_;
    double dead_reckoning_threshold_;
    double track_loss_timeout_sec_;
    std::string gripper_frame_;
    std::string base_frame_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace valve_inspection_fsm

#endif  // VALVE_INSPECTION_FSM__VALVE_CONVERGE_STATE_HPP_
