#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

namespace slalom_pole_finder {

/**
 * Estimates 3D slalom pole positions from 2D YOLO boxes and the known pole
 * length. Publishes one LandmarkArray per image, in the camera frame and with
 * the image stamp, so landmark_server transforms it with the vehicle pose at
 * the time the image was taken.
 */
class SlalomPoleFinderNode : public rclcpp::Node {
   public:
    SlalomPoleFinderNode();

   private:
    struct TrackedPole {
        geometry_msgs::msg::Point position;
        std::int32_t id;
    };

    void detectionCallback(
        vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);
    bool touchesImageEdge(const vision_msgs::msg::Detection2D& detection) const;
    std::int32_t assignId(const geometry_msgs::msg::Point& position,
                          std::unordered_set<std::size_t>& matched_tracks);

    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double object_height_;
    double edge_margin_px_;
    int image_width_;
    int image_height_;
    double match_distance_m_;
    std::string camera_frame_;
    std::string odom_frame_;
    double tf_timeout_s_;

    std::vector<TrackedPole> tracked_poles_;
    std::int32_t next_id_{1};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        detection_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_pub_;
};

}  // namespace slalom_pole_finder
