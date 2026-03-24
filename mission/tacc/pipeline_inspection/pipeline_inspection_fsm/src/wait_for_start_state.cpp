#include "pipeline_inspection_fsm/states.hpp"
#include "pipeline_inspection_fsm/param_utils.hpp"

#include <yasmin_ros/yasmin_node.hpp>

WaitForStartState::WaitForStartState(yasmin::Blackboard::SharedPtr)
    : TriggerWaitState(
          pipeline_inspection_fsm::param_utils::get_string(
              yasmin_ros::YasminNode::get_instance(),
              "services.start_mission")) {}
