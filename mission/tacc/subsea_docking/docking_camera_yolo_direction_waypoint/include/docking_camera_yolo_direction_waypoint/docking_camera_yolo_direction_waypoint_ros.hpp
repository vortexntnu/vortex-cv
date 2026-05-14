#ifndef DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT__DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT_ROS_HPP_
#define DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT__DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT_ROS_HPP_

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

#include <docking_camera_yolo_direction_waypoint/docking_camera_yolo_direction_waypoint.hpp>

namespace vortex::docking_camera_yolo_direction_waypoint {

/**
 * @brief ROS 2 node that computes an absolute pose waypoint from a YOLO
 * bounding box and sends it via the SendPose service.
 *
 * For each incoming detection the node:
 *   1. Picks the highest-confidence bounding box.
 *   2. Looks up odom->camera_frame via TF to get the camera's current position
 *      and heading in the odom frame.
 *   3. Computes the camera-relative yaw: atan2(dx, fx).
 *   4. Composes the absolute target yaw: yaw_camera_in_odom + yaw_camera_rel.
 *   5. Projects a waypoint at `waypoint_distance` metres along that heading
 *      from the camera's current position.
 *   6. Sends the waypoint pose via the SendPose service.
 *
 * Also publishes the target yaw as Float64 for debug/visualisation.
 */
class DockingCameraYoloDirectionWaypointNode : public rclcpp::Node {
   public:
    explicit DockingCameraYoloDirectionWaypointNode(
        const rclcpp::NodeOptions& options);

    ~DockingCameraYoloDirectionWaypointNode() = default;

   private:
    void setup_parameters();
    void setup_publishers_and_subscribers();

    /** @brief Store camera intrinsics from the first CameraInfo message. */
    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

    /**
     * @brief Main callback: pick best detection, compose absolute yaw,
     * project waypoint, call SendPose.
     */
    void detection_callback(
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& msg);

    /**
     * @brief Look up the odom->camera_frame transform at a given time.
     *
     * @return The transform, or nullopt if TF lookup fails.
     */
    std::optional<geometry_msgs::msg::TransformStamped>
    lookup_camera_transform_in_odom(const rclcpp::Time& stamp);

    /**
     * @brief Publish a landmark with the computed waypoint pose in the odom frame.
     *
     * @param x    Waypoint x in odom [m].
     * @param y    Waypoint y in odom [m].
     * @param yaw  Target heading in odom [rad].
     */
    void publish_landmark(double x, double y, double yaw);

    std::string detection_sub_topic_;
    std::string camera_info_sub_topic_;
    std::string landmarks_pub_topic_;
    std::string odom_frame_;
    std::string camera_frame_;   // derived from CameraInfo header, not a parameter
    double waypoint_distance_;

    std::optional<CameraIntrinsics> intrinsics_;

    YawExtractor extractor_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_sub_;

    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmarks_pub_;
};

}  // namespace vortex::docking_camera_yolo_direction_waypoint

#endif  // DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT__DOCKING_CAMERA_YOLO_DIRECTION_WAYPOINT_ROS_HPP_
