#include "vortex_yasmin_utils/landmark_polling_state.hpp"

#include <vortex_msgs/msg/landmark.hpp>
#include <yasmin_ros/basic_outcomes.hpp>

namespace vortex_yasmin_utils {

LandmarkPollingState::LandmarkPollingState(
    const std::string& action_server_name,
    vortex_msgs::msg::LandmarkType type,
    vortex_msgs::msg::LandmarkSubtype subtype)
    : ActionState(
          action_server_name,
          std::bind(&LandmarkPollingState::create_goal,
                    this,
                    std::placeholders::_1),
          yasmin::Outcomes{"landmark_found", yasmin_ros::basic_outcomes::ABORT},
          std::bind(&LandmarkPollingState::result_handler,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)),
      landmark_type_(type),
      landmark_subtype_(subtype) {}

LandmarkPollingAction::Goal LandmarkPollingState::create_goal(
    yasmin::Blackboard::SharedPtr /*blackboard*/) {
    LandmarkPollingAction::Goal goal;
    goal.type = landmark_type_;
    goal.subtype = landmark_subtype_;
    return goal;
}

std::string LandmarkPollingState::result_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    LandmarkPollingAction::Result::SharedPtr result) {
    if (result->landmarks.landmarks.empty())
        return yasmin_ros::basic_outcomes::ABORT;

    blackboard->set<bool>("landmark_found", true);
    blackboard->set<vortex_msgs::msg::Landmark>(
        "found_landmark", result->landmarks.landmarks.front());

    return "landmark_found";
}

}  // namespace vortex_yasmin_utils
