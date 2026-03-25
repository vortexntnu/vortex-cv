#include "pipeline_inspection_fsm/states.hpp"

#include <yasmin_ros/basic_outcomes.hpp>

LandmarkPollingState::LandmarkPollingState(
    yasmin::Blackboard::SharedPtr blackboard)
    : ActionState(
          blackboard->get<std::string>("action_server.landmark_polling"),
          std::bind(&LandmarkPollingState::create_goal,
                    this,
                    std::placeholders::_1),
          yasmin::Outcomes{"landmark_found", yasmin_ros::basic_outcomes::ABORT},
          std::bind(&LandmarkPollingState::result_handler,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)) {
    landmark_type_.value = 1;
    landmark_subtype_.value = 19;
}

pipeline_inspection_fsm::LandmarkPollingAction::Goal
LandmarkPollingState::create_goal(yasmin::Blackboard::SharedPtr blackboard) {
    pipeline_inspection_fsm::LandmarkPollingAction::Goal goal;

    (void)blackboard;

    goal.type = landmark_type_;
    goal.subtype = landmark_subtype_;

    return goal;
}

std::string LandmarkPollingState::result_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    pipeline_inspection_fsm::LandmarkPollingAction::Result::SharedPtr result) {
    if (result->landmarks.landmarks.empty())
        return yasmin_ros::basic_outcomes::ABORT;

    blackboard->set<bool>("landmark_found", true);
    blackboard->set<vortex_msgs::msg::Landmark>(
        "found_landmark", result->landmarks.landmarks.front());

    return "landmark_found";
}
