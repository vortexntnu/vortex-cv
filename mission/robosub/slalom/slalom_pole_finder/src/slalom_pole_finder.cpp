#include "slalom_pole_finder/slalom_pole_finder_node.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cmath>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>

namespace slalom_pole_finder {

namespace {
// spdlog has no throttle; warn at most once per 2 s.
void warn_throttled(const std::string& text) {
    static auto last = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (now - last > std::chrono::seconds(2)) {
        last = now;
        spdlog::warn("[slalom_pole_finder] {}", text);
    }
}
}  // namespace

SlalomPoleFinderNode::SlalomPoleFinderNode() : Node("slalom_pole_finder") {
    fx_ = declare_parameter<double>("fx", 1396.8086675);
    fy_ = declare_parameter<double>("fy", 1396.8086675);
    cx_ = declare_parameter<double>("cx", 960.0);
    cy_ = declare_parameter<double>("cy", 540.0);
    object_height_ = declare_parameter<double>("object_height", 0.9);
    edge_margin_px_ = declare_parameter<double>("edge_margin_px", 20.0);
    image_width_ = declare_parameter<int>("image_width", 1920);
    image_height_ = declare_parameter<int>("image_height", 1080);
    match_distance_m_ = declare_parameter<double>("match_distance_m", 0.75);
    // Used when the detections have no frame_id. The camera's pose comes
    // from TF, so the mount is set in the URDF, not here.
    camera_frame_ = declare_parameter<std::string>(
        "camera_frame", "nautilus/front_camera_color_optical");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "nautilus/odom");
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.1);

    if (fx_ <= 0.0 || fy_ <= 0.0 || object_height_ <= 0.0 ||
        !std::isfinite(fx_) || !std::isfinite(fy_) || !std::isfinite(cx_) ||
        !std::isfinite(cy_) || !std::isfinite(object_height_)) {
        throw std::invalid_argument(
            "Camera intrinsics and object_height must be finite and positive");
    }
    if (!std::isfinite(edge_margin_px_) || edge_margin_px_ < 0.0 ||
        image_width_ <= 0 || image_height_ <= 0 ||
        edge_margin_px_ * 2.0 >= image_width_ ||
        edge_margin_px_ * 2.0 >= image_height_ ||
        !std::isfinite(match_distance_m_) || match_distance_m_ <= 0.0) {
        throw std::invalid_argument(
            "Image size and edge/tracking parameters are invalid");
    }

    const std::string detections_topic = declare_parameter<std::string>(
        "detections_topic", "/yolo_object_detection/detections");
    const std::string positions_topic = declare_parameter<std::string>(
        "positions_topic", "/slalom_pole_finder/poles_3d");
    const std::string landmarks_topic = declare_parameter<std::string>(
        "landmarks_topic", "/nautilus/landmarks");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // Spins on its own thread, so waiting for a transform in the callback
    // does not block TF updates.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        detections_topic, 10,
        [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
            detectionCallback(msg);
        });
    pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseArray>(positions_topic, 10);
    landmark_pub_ =
        create_publisher<vortex_msgs::msg::LandmarkArray>(landmarks_topic, 10);

    spdlog::info("[slalom_pole_finder] Estimating 3D poles from '{}'",
                 detections_topic);
}

bool SlalomPoleFinderNode::touchesImageEdge(
    const vision_msgs::msg::Detection2D& detection) const {
    // A box clipped by any image boundary has an unreliable projected length.
    const double half_width = detection.bbox.size_x / 2.0;
    const double half_height = detection.bbox.size_y / 2.0;
    const double left = detection.bbox.center.position.x - half_width;
    const double right = detection.bbox.center.position.x + half_width;
    const double top = detection.bbox.center.position.y - half_height;
    const double bottom = detection.bbox.center.position.y + half_height;

    return left <= edge_margin_px_ || top <= edge_margin_px_ ||
           right >= static_cast<double>(image_width_) - edge_margin_px_ ||
           bottom >= static_cast<double>(image_height_) - edge_margin_px_;
}

std::int32_t SlalomPoleFinderNode::assignId(
    const geometry_msgs::msg::Point& position,
    std::unordered_set<std::size_t>& matched_tracks) {
    const double max_distance_squared = match_distance_m_ * match_distance_m_;
    double best_distance_squared = max_distance_squared;
    std::size_t best_index = tracked_poles_.size();

    for (std::size_t index = 0; index < tracked_poles_.size(); ++index) {
        if (matched_tracks.count(index) != 0) {
            continue;
        }
        const auto& previous = tracked_poles_[index].position;
        const double dx = position.x - previous.x;
        const double dy = position.y - previous.y;
        const double dz = position.z - previous.z;
        const double distance_squared = dx * dx + dy * dy + dz * dz;
        if (distance_squared <= best_distance_squared) {
            best_distance_squared = distance_squared;
            best_index = index;
        }
    }

    if (best_index == tracked_poles_.size()) {
        const std::int32_t id = next_id_++;
        tracked_poles_.push_back({position, id});
        matched_tracks.insert(tracked_poles_.size() - 1);
        return id;
    }

    auto& track = tracked_poles_[best_index];
    track.position = position;
    matched_tracks.insert(best_index);
    return track.id;
}

void SlalomPoleFinderNode::detectionCallback(
    vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
    const std::string camera_frame =
        msg->header.frame_id.empty() ? camera_frame_ : msg->header.frame_id;

    // Camera pose in odom at the image time: gives the vertical direction
    // for the tilt correction, and odom positions for the ids and the debug
    // PoseArray.
    geometry_msgs::msg::TransformStamped odom_camera;
    try {
        odom_camera = tf_buffer_->lookupTransform(
            odom_frame_, camera_frame, msg->header.stamp,
            rclcpp::Duration::from_seconds(tf_timeout_s_));
    } catch (const tf2::TransformException& ex) {
        warn_throttled("No transform " + odom_frame_ + " <- " + camera_frame +
                       " at the image time: " + ex.what());
        return;
    }

    const auto& r = odom_camera.transform.rotation;
    tf2::Quaternion q_odom_camera(r.x, r.y, r.z, r.w);
    if (q_odom_camera.length2() < 1e-12) {
        warn_throttled("Camera rotation is invalid");
        return;
    }
    q_odom_camera.normalize();
    const auto& t = odom_camera.transform.translation;
    const tf2::Vector3 t_odom_camera(t.x, t.y, t.z);

    // The odom frame's vertical pole axis in camera coordinates. Only the
    // vehicle tilt and the camera mount affect the projected length.
    const tf2::Vector3 pole_axis_camera =
        tf2::quatRotate(q_odom_camera.inverse(), tf2::Vector3(0.0, 0.0, 1.0));

    geometry_msgs::msg::PoseArray output;
    output.header.stamp = msg->header.stamp;
    output.header.frame_id = odom_frame_;
    vortex_msgs::msg::LandmarkArray landmark_output;
    landmark_output.header.stamp = msg->header.stamp;
    landmark_output.header.frame_id = camera_frame;

    std::unordered_set<std::size_t> matched_tracks;

    for (const auto& detection : msg->detections) {
        const double u = detection.bbox.center.position.x;
        const double v = detection.bbox.center.position.y;
        const double w_px = detection.bbox.size_x;
        const double h_px = detection.bbox.size_y;
        if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(w_px) ||
            !std::isfinite(h_px) || w_px <= 0.0 || h_px <= 1.0 ||
            touchesImageEdge(detection)) {
            continue;
        }

        if (detection.results.empty()) {
            continue;
        }

        std::uint16_t landmark_subtype;
        const std::string& class_id =
            detection.results.front().hypothesis.class_id;
        if (class_id == "0") {
            landmark_subtype =
                vortex_msgs::msg::LandmarkSubtype::SLALOM_PIPE_RED;
        } else if (class_id == "1") {
            landmark_subtype =
                vortex_msgs::msg::LandmarkSubtype::SLALOM_PIPE_WHITE;
        } else {
            warn_throttled("Ignoring unsupported YOLO class ID: " + class_id);
            continue;
        }

        // The diagonal approximates the projected length of a narrow pole
        // even when roll rotates it in the image. The projection scale below
        // corrects for pitch and for an off-axis pole.
        const double observed_length_px = std::hypot(w_px, h_px);
        const double normalized_x = (u - cx_) / fx_;
        const double normalized_y = (v - cy_) / fy_;
        const double projected_x =
            pole_axis_camera.x() - normalized_x * pole_axis_camera.z();
        const double projected_y =
            pole_axis_camera.y() - normalized_y * pole_axis_camera.z();
        const double projection_scale_px =
            std::hypot(fx_ * projected_x, fy_ * projected_y);
        if (!std::isfinite(projection_scale_px) || projection_scale_px < 1e-9) {
            continue;
        }

        // Solve the perspective projection of a pole centered on the
        // detection ray. The quadratic term matters when the pole axis has a
        // component in the camera-forward direction (the drone is pitched).
        const double half_depth_extent =
            0.5 * object_height_ * pole_axis_camera.z();
        const double scaled_height = object_height_ * projection_scale_px;
        const double discriminant = scaled_height * scaled_height +
                                    4.0 * observed_length_px *
                                        observed_length_px * half_depth_extent *
                                        half_depth_extent;
        const double depth = (scaled_height + std::sqrt(discriminant)) /
                             (2.0 * observed_length_px);
        if (!std::isfinite(depth) || depth <= std::abs(half_depth_extent)) {
            continue;
        }

        const tf2::Vector3 p_camera(normalized_x * depth, normalized_y * depth,
                                    depth);
        const tf2::Vector3 p_odom =
            tf2::quatRotate(q_odom_camera, p_camera) + t_odom_camera;

        geometry_msgs::msg::Pose odom_pose;
        odom_pose.position.x = p_odom.x();
        odom_pose.position.y = p_odom.y();
        odom_pose.position.z = p_odom.z();
        odom_pose.orientation.w = 1.0;
        output.poses.push_back(odom_pose);

        vortex_msgs::msg::Landmark landmark;
        landmark.header = landmark_output.header;
        landmark.id = assignId(odom_pose.position, matched_tracks);
        landmark.type.value = vortex_msgs::msg::LandmarkType::SLALOM_PIPE;
        landmark.subtype.value = landmark_subtype;
        landmark.pose.pose.position.x = p_camera.x();
        landmark.pose.pose.position.y = p_camera.y();
        landmark.pose.pose.position.z = p_camera.z();
        landmark.pose.pose.orientation.w = 1.0;
        // Position covariance not known yet: zeros let landmark_server use
        // its own noise model. Large rotation variance = position only.
        landmark.pose.covariance.fill(0.0);
        landmark.pose.covariance[21] = 1e6;
        landmark.pose.covariance[28] = 1e6;
        landmark.pose.covariance[35] = 1e6;
        landmark_output.landmarks.push_back(landmark);
    }

    pose_pub_->publish(output);
    landmark_pub_->publish(landmark_output);
}

}  // namespace slalom_pole_finder
