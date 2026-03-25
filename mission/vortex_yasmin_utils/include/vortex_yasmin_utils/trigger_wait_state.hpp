#ifndef VORTEX_YASMIN_UTILS__TRIGGER_WAIT_STATE_HPP_
#define VORTEX_YASMIN_UTILS__TRIGGER_WAIT_STATE_HPP_

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <yasmin/state.hpp>

namespace vortex_yasmin_utils {

/**
 * @brief A YASMIN state that blocks until a std_srvs/Trigger service is called.
 *
 * Creates a ROS service server and waits for it to be triggered.
 * Returns SUCCEED when the service is called, CANCEL if the state is canceled,
 * or TIMEOUT if the optional timeout expires.
 *
 * @param service_name  The fully-qualified ROS service name to advertise.
 * @param timeout       How long to wait before timing out.
 *                      A duration of zero (default) means wait indefinitely.
 */
class TriggerWaitState : public yasmin::State {
   public:
    explicit TriggerWaitState(const std::string& service_name,
                              std::chrono::duration<double> timeout =
                                  std::chrono::duration<double>(0));

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
    void cancel_state() override;

    static constexpr const char* TIMEOUT = "timeout";

   protected:
    virtual void on_triggered(yasmin::Blackboard::SharedPtr blackboard);

   private:
    void service_callback(std_srvs::srv::Trigger::Request::SharedPtr req,
                          std_srvs::srv::Trigger::Response::SharedPtr res);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    std::condition_variable cv_;
    std::mutex mutex_;
    bool triggered_{false};
    std::chrono::duration<double> timeout_;
};

}  // namespace vortex_yasmin_utils

#endif  // VORTEX_YASMIN_UTILS__TRIGGER_WAIT_STATE_HPP_
