#ifndef VORTEX_YASMIN_UTILS__HELLO_STATE_HPP_
#define VORTEX_YASMIN_UTILS__HELLO_STATE_HPP_

#include <string>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>

namespace vortex_yasmin_utils {

/**
 * @brief Minimal example state: logs a message and returns SUCCEED.
 *
 * This is the learning_period onboarding mission's "write your own state"
 * exercise. It takes no constructor arguments and touches no ROS
 * infrastructure, so only execute() is left to fill in.
 *
 * Outcomes: SUCCEED.
 */
class HelloState : public yasmin::State {
   public:
    HelloState();
    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__HELLO_STATE_HPP_
