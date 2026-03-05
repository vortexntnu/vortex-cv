#include <cv_bridge/cv_bridge.h>
#include <spdlog/spdlog.h>
#include "line_detection_houghp/lib/mode_utils.hpp"
#include "line_detection_houghp/ros/line_detection_houghp_ros.hpp"

namespace vortex::line_detection {

static bool starts_with(const std::string& s, const std::string& prefix) {
  return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

void LineDetectionHoughPNode::initialize_parameter_handler() {
  param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

  auto parameter_event_callback =
      [this](const rcl_interfaces::msg::ParameterEvent& event) -> void {
    this->on_parameter_event(event);
  };

  param_cb_handle_ =
      param_handler_->add_parameter_event_callback(parameter_event_callback);
}

void LineDetectionHoughPNode::on_parameter_event(
    const rcl_interfaces::msg::ParameterEvent& event) {
  const auto node_name = this->get_fully_qualified_name();
  if (event.node != node_name)
    return;

  bool detector_params_changed = false;
  bool mode_changed = false;

  for (const auto& p : event.changed_parameters) {
    const auto& name = p.name;

    if (starts_with(name, "canny.") || starts_with(name, "houghp.")) {
      detector_params_changed = true;
    } else if (name == "mode") {
      mode_changed = true;
    } else {
      continue;
    }

    spdlog::info("Parameter changed: {}", name);
  }

  if (mode_changed) {
    set_mode();
  }

  if (detector_params_changed) {
    set_detector();
    spdlog::info("Recreated HoughPLineDetector with updated params");
  }
}

void LineDetectionHoughPNode::mode_conditional_publishing(
    const Result& result,
    const std_msgs::msg::Header& header) {
  switch (mode_) {
    case DetectorMode::standard:
      return;

    case DetectorMode::visualize: {
      const auto* vis = std::get_if<VisualizeOutput>(&result.output);
      if (!vis) {
        spdlog::warn(
            "Mode is visualize but Result::output is not "
            "VisualizeOutput");
        return;
      }

      cv_bridge::CvImage overlay_msg;
      overlay_msg.header = header;
      overlay_msg.encoding = sensor_msgs::image_encodings::BGR8;
      overlay_msg.image = vis->overlay_color;  // expected CV_8UC3
      color_overlay_pub_->publish(*overlay_msg.toImageMsg());
      return;
    }

    case DetectorMode::debug: {
      const auto* dbg = std::get_if<DebugOutput>(&result.output);
      if (!dbg) {
        spdlog::warn("Mode is debug but Result::output is not DebugOutput");
        return;
      }

      // overlay_canny: CV_8UC3 -> bgr8
      {
        cv_bridge::CvImage m;
        m.header = header;
        m.encoding = sensor_msgs::image_encodings::BGR8;
        m.image = dbg->overlay_canny;
        canny_overlay_pub_->publish(*m.toImageMsg());
      }

      // canny: CV_8UC1 -> mono8
      {
        cv_bridge::CvImage m;
        m.header = header;
        m.encoding = sensor_msgs::image_encodings::MONO8;
        m.image = dbg->canny;
        canny_debug_pub_->publish(*m.toImageMsg());
      }

      // overlay_color: CV_8UC3 -> bgr8
      {
        cv_bridge::CvImage m;
        m.header = header;
        m.encoding = sensor_msgs::image_encodings::BGR8;
        m.image = dbg->overlay_color;
        color_overlay_pub_->publish(*m.toImageMsg());
      }

      return;
    }

    default:
      return;
  }
}

void LineDetectionHoughPNode::set_mode() {
  const auto mode_str = this->get_parameter("mode").as_string();

  try {
    mode_ = parse_mode(mode_str);
    spdlog::info("Set detector mode to '{}'", mode_str);
  } catch (const std::exception& e) {
    spdlog::warn("Invalid mode '{}': {}. Keeping previous mode.", mode_str,
                 e.what());
  }
}

}  // namespace vortex::line_detection
