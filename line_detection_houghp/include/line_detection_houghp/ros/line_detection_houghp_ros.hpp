#ifndef LINE_DETECTION_HOUGHP__ROS__LINE_DETECTION_HOUGHP_ROS_HPP_
#define LINE_DETECTION_HOUGHP__ROS__LINE_DETECTION_HOUGHP_ROS_HPP_

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vortex_msgs/msg/line_segment2_d_array.hpp>
#include "line_detection_houghp/lib/line_detection_houghp.hpp"

namespace vortex::line_detection {

class LineDetectionHoughPNode : public rclcpp::Node {
   public:
    explicit LineDetectionHoughPNode(const rclcpp::NodeOptions& options);

    ~LineDetectionHoughPNode() override = default;

   private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<vortex_msgs::msg::LineSegment2DArray>::SharedPtr
        line_segments_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_overlay_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr canny_debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr canny_overlay_pub_;

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    void declare_parameters();

    void setup_publishers_and_subscribers();

    void set_detector();

    void set_mode();

    void mode_conditional_publishing(const Result& result,
                                     const std_msgs::msg::Header& header);

    /**
     * @brief Initialize the parameter handler and a parameter event callback.
     *
     */
    void initialize_parameter_handler();
    /**
     * @brief Callback function for parameter events.
     * Checks for parameter changes that matches the nodes' namespace and
     * invokes the relevant initializer functions to update member variables.
     *
     * @param event The parameter event.
     */
    void on_parameter_event(const rcl_interfaces::msg::ParameterEvent& event);

    /**
     * @brief Manages parameter events for the node.
     *
     * This handle is used to set up a mechanism to listen for and react to
     * changes in parameters. Parameters can be used to configure the node's
     * operational behavior dynamically, allowing adjustments without altering
     * the code. The `param_handler_` is responsible for registering callbacks
     * that are triggered on parameter changes, providing a centralized
     * management system within the node for such events.
     */
    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;

    /**
     * @brief Handle to the registration of the parameter event callback.
     *
     * Represents a token or reference to the specific callback registration
     * made with the parameter event handler (`param_handler_`). This handle
     * allows for management of the lifecycle of the callback, such as removing
     * the callback if it's no longer needed. It ensures that the node can
     * respond to parameter changes with the registered callback in an efficient
     * and controlled manner.
     */
    rclcpp::ParameterEventCallbackHandle::SharedPtr param_cb_handle_;

    std::unique_ptr<LineDetectorHoughP> detector_{};

    DetectorMode mode_{DetectorMode::standard};
};

}  // namespace vortex::line_detection

#endif  // LINE_DETECTION_HOUGHP__ROS__LINE_DETECTION_HOUGHP_ROS_HPP_
