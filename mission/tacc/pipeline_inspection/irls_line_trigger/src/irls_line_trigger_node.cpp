// Bridges the IRLS line fitter to the pipeline inspection FSM.
//
// Watches the IRLS line topic and, as soon as a line segment is published,
// calls a std_srvs/Trigger service that the FSM's CONVERGE state is waiting on.
// Because the IRLS fitter is already gated by a minimum segment length, the
// presence of any line is a robust "the pipe is clearly in view" signal, so we
// use it to leave convergence early instead of waiting for the (likely
// overshooting) convergence waypoint to be reached.
//
// The trigger is fired on every qualifying message (throttled), not just the
// first one, so that whenever the FSM happens to enter its wait state the next
// camera frame re-triggers it -- there is no single event that can be missed.

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <vortex_msgs/msg/line_segment2_d_array.hpp>

using namespace std::chrono_literals;

class IrlsLineTriggerNode : public rclcpp::Node {
   public:
    IrlsLineTriggerNode() : Node("irls_line_trigger_node") {
        input_topic_ =
            declare_parameter<std::string>("input_topic", "/irls_line/lines");
        service_name_ = declare_parameter<std::string>(
            "service_name", "pipeline_inspection_fsm/irls_line_detected");
        // Minimum spacing between service calls, to avoid spamming the FSM at
        // full camera rate while still re-firing often enough to be caught.
        min_call_interval_ = std::chrono::duration<double>(
            declare_parameter<double>("min_call_interval_sec", 0.5));

        client_ = create_client<std_srvs::srv::Trigger>(service_name_);

        sub_ = create_subscription<vortex_msgs::msg::LineSegment2DArray>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&IrlsLineTriggerNode::lines_cb, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "IRLS line trigger ready: watching '%s', will call '%s'",
                    input_topic_.c_str(), service_name_.c_str());
    }

   private:
    void lines_cb(const vortex_msgs::msg::LineSegment2DArray::SharedPtr msg) {
        if (msg->lines.empty())
            return;

        const auto now = std::chrono::steady_clock::now();
        if (now - last_call_ < min_call_interval_)
            return;
        last_call_ = now;

        if (!client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "IRLS line seen but service '%s' not available",
                service_name_.c_str());
            return;
        }

        if (!logged_first_) {
            RCLCPP_INFO(get_logger(),
                        "First IRLS line detected -- triggering pipeline "
                        "following transition");
            logged_first_ = true;
        }

        client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
    }

    std::string input_topic_;
    std::string service_name_;
    std::chrono::duration<double> min_call_interval_{0.5};
    std::chrono::steady_clock::time_point last_call_{};
    bool logged_first_{false};

    rclcpp::Subscription<vortex_msgs::msg::LineSegment2DArray>::SharedPtr sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IrlsLineTriggerNode>());
    rclcpp::shutdown();
    return 0;
}
