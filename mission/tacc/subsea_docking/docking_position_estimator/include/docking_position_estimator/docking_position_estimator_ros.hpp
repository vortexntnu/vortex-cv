#ifndef DOCKING_POSITION_ESTIMATOR__DOCKING_POSITION_ESTIMATOR_ROS_HPP_
#define DOCKING_POSITION_ESTIMATOR__DOCKING_POSITION_ESTIMATOR_ROS_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <vortex_msgs/msg/line_segment2_d_array.hpp>
#include <vortex_msgs/msg/sonar_info.hpp>
#include <vortex_msgs/srv/send_pose.hpp>

#include <vortex/utils/types.hpp>

#include <docking_position_estimator/docking_position_estimator.hpp>
#include <visualization_msgs/msg/marker.hpp>  // til testing

namespace vortex::docking_position_estimator {

/**
 * @brief ROS 2 node for estimating a docking position from sonar line
 * detections.
 *
 * This node provides the ROS interface for the docking position estimator.
 * It subscribes to detected sonar line segments, drone pose, and sonar
 * image metadata, transforms the incoming line segments into the odometry
 * frame, and classifies them as candidate walls relative to the drone.
 *
 * From the classified walls, the node estimates corner candidates by
 * intersecting right-wall or left-wall and far-wall segments. The most
 * suitable corner is selected, and a docking position is computed by
 * offsetting from the corner along the wall normals.
 *
 * If a valid docking estimate is found, it is sent through the SendPose
 * service.
 */
class DockingPositionEstimatorNode : public rclcpp::Node {
   public:
    /**
     * @brief Construct a new DockingPositionEstimatorNode.
     *
     * The constructor initializes parameters, ROS interfaces, and the
     * estimator.
     *
     * @param options ROS node options.
     */
    explicit DockingPositionEstimatorNode(const rclcpp::NodeOptions& options);

    /** @brief Default destructor. */
    ~DockingPositionEstimatorNode() = default;

   private:
    // setup

    /**
     * @brief Declare and load ROS parameters used by the node.
     *
     * This includes frame names, topic names, and waypoint-related parameters.
     */
    void setup_parameters();

    /**
     * @brief Create subscribers, TF utilities, and service clients.
     *
     * This method initializes:
     * - TF buffer and listener,
     * - line, pose, and sonar-info subscriptions,
     * - TF message filter for line detections,
     * - docking pose service client.
     */
    void setup_publishers_and_subscribers();

    /**
     * @brief Create and configure the docking position estimator.
     *
     * Estimator configuration values are loaded from ROS parameters and used to
     * construct the internal estimator instance.
     */
    void setup_estimator();

    // callbacks

    /**
     * @brief Callback for drone pose updates.
     *
     * Updates the internal drone state estimate used by the estimator.
     *
     * @param msg Drone pose message with covariance.
     */
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr&
            msg);

    /**
     * @brief Callback for sonar image metadata updates.
     *
     * Stores the most recent sonar information needed to convert detected line
     * endpoints from image pixels to metric sonar coordinates.
     *
     * @param msg Sonar metadata message.
     */
    void sonar_info_callback(
        const vortex_msgs::msg::SonarInfo::ConstSharedPtr& msg);

    /**
     * @brief Callback for detected line segments.
     *
     * Triggered when a line segment message passes the TF message filter. The
     * callback attempts to estimate a docking position from the detected lines.
     *
     * @param msg Array of detected 2D line segments.
     */
    void line_callback(
        const vortex_msgs::msg::LineSegment2DArray::ConstSharedPtr& msg);

    // helpers

    /**
     * @brief Estimate and send a docking position from detected line segments.
     *
     * The input segments are transformed into the odometry frame, passed to the
     * estimator for wall classification and corner detection, and then used to
     * compute a docking position. If a valid estimate is found, it is sent
     * through the SendPose service.
     *
     * @param msg Array of detected 2D line segments.
     */
    void estimate_and_send_docking_waypoint(
        const vortex_msgs::msg::LineSegment2DArray& msg);

    /**
     * @brief Transform sonar-detected 2D segments into a target frame.
     *
     * Each line endpoint is first converted from pixel coordinates to sonar
     * metric coordinates using the latest sonar metadata. The resulting points
     * are then transformed by the provided homogeneous transform.
     *
     * @param msg Input line segment array.
     * @param T_target_src Homogeneous transform from source frame to target
     * frame.
     * @return Vector of transformed 2D line segments in the target frame.
     */
    std::vector<vortex::utils::types::LineSegment2D> transform_segments_2d(
        const vortex_msgs::msg::LineSegment2DArray& msg,
        const Eigen::Matrix4f& T_target_src);

    /**
     * @brief Send a docking position estimate through the SendPose service.
     *
     * A single pose is created from the estimated docking position and sent
     * in the odometry frame.
     *
     * @param docking_estimate Estimated docking position in the odometry frame.
     */
    void send_docking_waypoint(const Eigen::Vector2f& docking_estimate);

    // TIL TESTING
    void publish_docking_marker(const Eigen::Vector2f& docking);

    /** @brief Topic name for detected sonar line segments. */
    std::string line_sub_topic_;

    /** @brief Topic name for vehicle pose estimates. */
    std::string pose_sub_topic_;

    /** @brief Topic name for sonar image metadata. */
    std::string sonar_info_sub_topic_;

    // TIL TESTING
    std::string debug_topic_;

    /** @brief Odometry frame used as reference frame for estimation. */
    std::string odom_frame_;

    /** @brief True once a docking pose has been sent. */
    bool waypoint_sent_{false};

    /** @brief Latest known vehicle pose and heading used by the estimator. */
    vortex::utils::types::PoseEuler drone_state_;

    /** @brief Most recent sonar metadata message. */
    vortex_msgs::msg::SonarInfo::ConstSharedPtr latest_sonar_info_;

    /** @brief TF buffer used for transform lookup. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /** @brief TF listener attached to the TF buffer. */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /** @brief Subscriber for line segment detections. */
    message_filters::Subscriber<vortex_msgs::msg::LineSegment2DArray> line_sub_;

    /**
     * @brief TF-aware message filter for line segment detections.
     *
     * Ensures that line messages are only forwarded once the required transform
     * into the target frame is available.
     */
    std::shared_ptr<
        tf2_ros::MessageFilter<vortex_msgs::msg::LineSegment2DArray>>
        line_filter_;

    /** @brief Subscriber for vehicle pose updates. */
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    /** @brief Subscriber for sonar metadata updates. */
    rclcpp::Subscription<vortex_msgs::msg::SonarInfo>::SharedPtr
        sonar_info_sub_;

    /** @brief Client for sending the docking pose estimate. */
    rclcpp::Client<vortex_msgs::srv::SendPose>::SharedPtr send_pose_client_;

    /** @brief Docking position estimator used for corner detection and docking
     * estimation. */
    std::unique_ptr<DockingPositionEstimator> estimator_;

    // Debug visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        docking_marker_pub_;  // til testing
};

}  // namespace vortex::docking_position_estimator

#endif  // DOCKING_POSITION_ESTIMATOR__DOCKING_POSITION_ESTIMATOR_ROS_HPP_
