#ifndef VORTEX_YASMIN_UTILS__WIPE_STATE_HPP_
#define VORTEX_YASMIN_UTILS__WIPE_STATE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin/state.hpp>

namespace vortex_yasmin_utils {

/**
 * @brief Publishes to mission/wipe to reset the reference filter,
 * landmark server, and waypoint manager at mission start.
 *
 * Returns SUCCEED immediately after publishing.
 */
class WipeState : public yasmin::State {
   public:
    WipeState();
    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

   private:
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__WIPE_STATE_HPP_
