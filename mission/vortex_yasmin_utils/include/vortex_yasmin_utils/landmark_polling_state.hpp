#ifndef VORTEX_YASMIN_UTILS__LANDMARK_POLLING_STATE_HPP_
#define VORTEX_YASMIN_UTILS__LANDMARK_POLLING_STATE_HPP_

#include <string>

#include <vortex_msgs/action/landmark_polling.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <yasmin_ros/action_state.hpp>

namespace vortex_yasmin_utils {

using LandmarkPollingAction = vortex_msgs::action::LandmarkPolling;

/**
 * @brief Polls for landmarks of a given type/subtype.
 *
 * On success, sets "landmark_found" (bool) and "found_landmark"
 * (vortex_msgs::msg::Landmark) on the blackboard.
 *
 * Outcomes: "landmark_found", ABORT.
 */
class LandmarkPollingState
    : public yasmin_ros::ActionState<LandmarkPollingAction> {
   public:
    LandmarkPollingState(const std::string& action_server_name,
                         vortex_msgs::msg::LandmarkType type,
                         vortex_msgs::msg::LandmarkSubtype subtype);

    LandmarkPollingAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

    std::string result_handler(yasmin::Blackboard::SharedPtr blackboard,
                               LandmarkPollingAction::Result::SharedPtr result);

   private:
    vortex_msgs::msg::LandmarkType landmark_type_;
    vortex_msgs::msg::LandmarkSubtype landmark_subtype_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__LANDMARK_POLLING_STATE_HPP_
