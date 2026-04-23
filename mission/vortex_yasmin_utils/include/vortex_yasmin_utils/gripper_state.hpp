#ifndef VORTEX_YASMIN_UTILS__GRIPPER_STATE_HPP_
#define VORTEX_YASMIN_UTILS__GRIPPER_STATE_HPP_

#include <string>

#include <vortex_msgs/action/gripper_reference_filter_waypoint.hpp>

#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using GripperAction = vortex_msgs::action::GripperReferenceFilterWaypoint;

/**
 * @brief Sends a gripper waypoint to the GripperReferenceFilter action server.
 *
 * Configures roll, pinch, mode, and convergence threshold at construction time.
 * Returns SUCCEED if the action server reports success, ABORT otherwise.
 * No blackboard reads or writes — safe inside a yasmin concurrent container.
 *
 * Outcomes: SUCCEED, ABORT.
 *
 * @param action_server_name     Name of the GripperReferenceFilter action server.
 * @param roll                   Target roll value (radians).
 * @param pinch                  Target pinch value (normalised 0–1).
 * @param mode                   GripperWaypoint mode (ROLL_AND_PINCH=0,
 *                               ONLY_ROLL=1, ONLY_PINCH=2).
 * @param convergence_threshold  Threshold below which the goal is considered
 *                               reached.
 */
class GripperState : public yasmin_ros::ActionState<GripperAction> {
   public:
    GripperState(const std::string& action_server_name,
                 double roll,
                 double pinch,
                 uint8_t mode,
                 double convergence_threshold);

    GripperAction::Goal create_goal(yasmin::Blackboard::SharedPtr blackboard);

    std::string result_handler(yasmin::Blackboard::SharedPtr blackboard,
                               GripperAction::Result::SharedPtr result);

    void on_feedback(yasmin::Blackboard::SharedPtr blackboard,
                     std::shared_ptr<const GripperAction::Feedback> feedback);

   private:
    double roll_;
    double pinch_;
    uint8_t mode_;
    double convergence_threshold_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__GRIPPER_STATE_HPP_
