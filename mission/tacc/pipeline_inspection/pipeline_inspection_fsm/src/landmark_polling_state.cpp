#include "pipeline_inspection_fsm/states.hpp"
#include "pipeline_inspection_fsm/param_utils.hpp"

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/yasmin_node.hpp>

LandmarkPollingState::LandmarkPollingState(yasmin::Blackboard::SharedPtr)
    : ActionState(
          pipeline_inspection_fsm::param_utils::get_string(
              yasmin_ros::YasminNode::get_instance(),
              "action_servers.landmark_polling"),
          std::bind(&LandmarkPollingState::create_goal,
                    this,
                    std::placeholders::_1),
          yasmin::Outcomes{"landmark_found", yasmin_ros::basic_outcomes::ABORT},
          std::bind(&LandmarkPollingState::result_handler,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)) {
    auto node = yasmin_ros::YasminNode::get_instance();

    landmark_type_ = static_cast<int8_t>(
        pipeline_inspection_fsm::param_utils::get_int(node, "fsm.landmark_type"));
    landmark_subtype_ = static_cast<int8_t>(
        pipeline_inspection_fsm::param_utils::get_int(node, "fsm.landmark_subtype"));
}

pipeline_inspection_fsm::LandmarkPollingAction::Goal LandmarkPollingState::create_goal(
    yasmin::Blackboard::SharedPtr blackboard) {
    pipeline_inspection_fsm::LandmarkPollingAction::Goal goal;

    (void)blackboard;

    goal.type.value = landmark_type_;
    goal.subtype.value = landmark_subtype_;

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
