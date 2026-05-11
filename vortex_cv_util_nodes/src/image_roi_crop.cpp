#include "vortex_cv_util_nodes/image_roi_crop.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace vortex_cv_util_nodes {

ImageRoiCrop::ImageRoiCrop(const rclcpp::NodeOptions& options)
    : Node("image_roi_crop", options) {
    const auto image_topic = declare_parameter<std::string>("image_topic");
    const auto info_topic = declare_parameter<std::string>("camera_info_topic");
    const auto out_image_topic =
        declare_parameter<std::string>("output_image_topic");
    const auto out_info_topic =
        declare_parameter<std::string>("output_camera_info_topic");

    x_ = declare_parameter<int>("crop.x_offset");
    y_ = declare_parameter<int>("crop.y_offset");
    w_ = declare_parameter<int>("crop.width");
    h_ = declare_parameter<int>("crop.height");
    enable_crop_ = declare_parameter<bool>("enable_crop");

    const auto sensor_data_qos = rclcpp::QoS(10).best_effort();

    image_pub_ = create_publisher<ImageMsg>(out_image_topic, sensor_data_qos);
    info_pub_ =
        create_publisher<CameraInfoMsg>(out_info_topic, sensor_data_qos);

    image_sub_.subscribe(this, image_topic,
                         sensor_data_qos.get_rmw_qos_profile());
    info_sub_.subscribe(this, info_topic,
                        sensor_data_qos.get_rmw_qos_profile());

    sync_ =
        std::make_shared<Synchronizer>(SyncPolicy(10), image_sub_, info_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
    sync_->registerCallback(&ImageRoiCrop::callback, this);

    if (enable_crop_) {
        RCLCPP_INFO(get_logger(),
                    "image_roi_crop: %s -> %s [x=%d, y=%d, w=%s, h=%s]",
                    image_topic.c_str(), out_image_topic.c_str(), x_, y_,
                    w_ > 0 ? std::to_string(w_).c_str() : "full",
                    h_ > 0 ? std::to_string(h_).c_str() : "full");
    } else {
        RCLCPP_INFO(get_logger(), "image_roi_crop: passthrough %s -> %s",
                    image_topic.c_str(), out_image_topic.c_str());
    }
}

void ImageRoiCrop::callback(const ImageMsg::ConstSharedPtr& image_msg,
                            const CameraInfoMsg::ConstSharedPtr& info_msg) {
    if (!enable_crop_) {
        image_pub_->publish(*image_msg);
        info_pub_->publish(*info_msg);
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(image_msg);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    const int img_h = cv_ptr->image.rows;
    const int img_w = cv_ptr->image.cols;
    const int w = (w_ > 0) ? w_ : img_w - x_;
    const int h = (h_ > 0) ? h_ : img_h - y_;

    cv::Mat cropped = cv_ptr->image(cv::Rect(x_, y_, w, h));

    auto out_image =
        cv_bridge::CvImage(image_msg->header, image_msg->encoding, cropped)
            .toImageMsg();
    image_pub_->publish(*out_image);

    CameraInfoMsg out_info = *info_msg;
    out_info.width = static_cast<uint32_t>(w);
    out_info.height = static_cast<uint32_t>(h);
    out_info.k[2] -= x_;  // cx
    out_info.k[5] -= y_;  // cy
    out_info.p[2] -= x_;  // cx
    out_info.p[6] -= y_;  // cy
    out_info.roi.x_offset = static_cast<uint32_t>(x_);
    out_info.roi.y_offset = static_cast<uint32_t>(y_);
    out_info.roi.width = static_cast<uint32_t>(w);
    out_info.roi.height = static_cast<uint32_t>(h);
    out_info.roi.do_rectify = false;

    info_pub_->publish(out_info);
}

}  // namespace vortex_cv_util_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(vortex_cv_util_nodes::ImageRoiCrop)
