#include <cv_bridge/cv_bridge.h>
#include <spdlog/spdlog.h>
#include "line_detection_ransac/lib/mode_utils.hpp"
#include "line_detection_ransac/ros/line_detection_ransac_ros.hpp"

namespace vortex::line_detection {

void LineDetectionRansacNode::set_mode() {
  const auto mode_str = this->get_parameter("mode").as_string();

  try {
    mode_ = parse_mode(mode_str);
    spdlog::info("Set detector mode to '{}'", mode_str);
  } catch (const std::exception& e) {
    spdlog::warn("Invalid mode '{}': {}. Keeping previous mode.", mode_str,
                 e.what());
  }
}

void LineDetectionRansacNode::mode_conditional_publishing(
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

      // overlay_boundaries: CV_8UC3 -> bgr8
      {
        cv_bridge::CvImage m;
        m.header = header;
        m.encoding = sensor_msgs::image_encodings::BGR8;
        m.image = dbg->overlay_boundaries;
        boundary_overlay_pub_->publish(*m.toImageMsg());
      }

      // boundaries: CV_8UC1 -> bgr8
      {
        cv_bridge::CvImage m;
        m.header = header;
        m.encoding = sensor_msgs::image_encodings::BGR8;
        m.image = dbg->boundaries;
        boundary_debug_pub_->publish(*m.toImageMsg());
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

}  // namespace vortex::line_detection
