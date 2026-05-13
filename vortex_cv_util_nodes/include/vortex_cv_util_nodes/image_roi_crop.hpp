#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace vortex_cv_util_nodes {

class ImageRoiCrop : public rclcpp::Node {
   public:
    explicit ImageRoiCrop(const rclcpp::NodeOptions& options);

   private:
    using ImageMsg = sensor_msgs::msg::Image;
    using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<ImageMsg,
                                                        CameraInfoMsg>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    void callback(const ImageMsg::ConstSharedPtr& image_msg,
                  const CameraInfoMsg::ConstSharedPtr& info_msg);

    rclcpp::Publisher<ImageMsg>::SharedPtr image_pub_;
    rclcpp::Publisher<CameraInfoMsg>::SharedPtr info_pub_;

    message_filters::Subscriber<ImageMsg> image_sub_;
    message_filters::Subscriber<CameraInfoMsg> info_sub_;
    std::shared_ptr<Synchronizer> sync_;

    int x_{0}, y_{0}, w_{0}, h_{0};
    bool enable_crop_{true};
    // If non-empty, overrides `header.frame_id` on the published image and
    // camera_info. Empty = preserve the incoming frame.
    std::string output_frame_;
};

}  // namespace vortex_cv_util_nodes
