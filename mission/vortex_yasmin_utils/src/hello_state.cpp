#include "vortex_yasmin_utils/hello_state.hpp"

#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>

namespace vortex_yasmin_utils {

HelloState::HelloState()
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}) {}

std::string HelloState::execute(yasmin::Blackboard::SharedPtr /*blackboard*/) {
    // TODO(member): log a greeting with YASMIN_LOG_INFO (from
    // <yasmin_ros/ros_logs.hpp>, already included above), then return
    // yasmin_ros::basic_outcomes::SUCCEED. For the simplest possible
    // execute() body to copy the shape of, see WipeState::execute() in
    // wipe_state.cpp in this same directory.
    return yasmin_ros::basic_outcomes::SUCCEED;
}

}  // namespace vortex_yasmin_utils
