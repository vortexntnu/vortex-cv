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
 * On success, stores the full vector of landmarks on the blackboard
 * under the caller-specified key.
 *
 * Outcomes: caller-specified success outcome, ABORT.
 *
 * @param action_server_name  Name of the LandmarkPolling action server.
 * @param type                Landmark type to poll for.
 * @param subtype             Landmark subtype to poll for.
 * @param landmarks_bb_key    Blackboard key for the output landmarks vector.
 * @param success_outcome     Outcome string returned on success.
 */
class LandmarkPollingState
    : public yasmin_ros::ActionState<LandmarkPollingAction> {
   public:
    LandmarkPollingState(
        const std::string& action_server_name,
        vortex_msgs::msg::LandmarkType type,
        vortex_msgs::msg::LandmarkSubtype subtype,
        const std::string& landmarks_bb_key,
        const std::string& success_outcome = "landmarks_found");

    LandmarkPollingAction::Goal create_goal(
        yasmin::Blackboard::SharedPtr blackboard);

    std::string result_handler(yasmin::Blackboard::SharedPtr blackboard,
                               LandmarkPollingAction::Result::SharedPtr result);

   private:
    vortex_msgs::msg::LandmarkType landmark_type_;
    vortex_msgs::msg::LandmarkSubtype landmark_subtype_;
    std::string landmarks_bb_key_;
    std::string success_outcome_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__LANDMARK_POLLING_STATE_HPP_
