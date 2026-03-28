#ifndef VALVE_DETECTION_ROS__VALVE_POSE_ROS_HPP_
#define VALVE_DETECTION_ROS__VALVE_POSE_ROS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "valve_detection/detection_utils.hpp"
#include "valve_detection/pcl_extraction.hpp"
#include "valve_detection/pose_estimator.hpp"
#include "valve_detection/types.hpp"

#include <memory>
#include <string>
#include <vector>
#include "vortex_msgs/msg/landmark_array.hpp"
#include "vortex_msgs/msg/landmark_subtype.hpp"
#include "vortex_msgs/msg/landmark_type.hpp"

namespace valve_detection {

class ValvePoseNode : public rclcpp::Node {
   public:
    explicit ValvePoseNode(const rclcpp::NodeOptions& options);

   private:
    // Node setup — called from constructor.
    void declare_params();
    void init_subscriptions();
    void try_activate_detector();
    void lookup_extrinsic();

    // Camera info callbacks (one-shot, override config fallback).
    void color_camera_info_cb(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void depth_camera_info_cb(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // Main synchronized callback: depth + detections.
    void sync_cb(const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                 const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det);

    // sync_cb helpers
    std::vector<std::pair<float, BoundingBox>> collect_scored_boxes(
        const vision_msgs::msg::Detection2DArray& det) const;
    void publish_empty_results(const std_msgs::msg::Header& header) const;
    cv::Mat build_depth_colormap(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth) const;
    void publish_debug(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth,
        const std::vector<BoundingBox>& boxes,
        const std::vector<Pose>& poses,
        const pcl::PointCloud<pcl::PointXYZ>& ann_cloud,
        const pcl::PointCloud<pcl::PointXYZ>& pln_cloud) const;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        vision_msgs::msg::Detection2DArray>;

    // params
    bool debug_visualize_;
    float iou_duplicate_threshold_;
    std::string output_frame_id_;

    // estimator config params (stored for deferred detector init)
    int yolo_w_, yolo_h_;
    float annulus_ratio_, ransac_thresh_, handle_offset_;
    int ransac_iters_;

    // camera data (owned by node, passed to estimator and depth functions)
    ImageProperties color_props_{};
    ImageProperties depth_props_{};
    DepthColorExtrinsic depth_color_extrinsic_{};
    bool color_props_ready_{false};
    bool depth_props_ready_{false};
    bool extrinsic_ready_{false};

    // TF2 lookup for depth-to-color extrinsic
    std::string depth_frame_id_;
    std::string color_frame_id_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr extrinsic_timer_;

    // estimator
    std::unique_ptr<PoseEstimator> detector_;

    // subs
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        color_cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        depth_cam_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // pubs
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_colormap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr annulus_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        depth_cloud_pub_;

    float depth_colormap_vmin_;
    float depth_colormap_vmax_;
};

}  // namespace valve_detection

#endif  // VALVE_DETECTION_ROS__VALVE_POSE_ROS_HPP_
