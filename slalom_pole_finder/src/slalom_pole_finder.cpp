#include "slalom_pole_finder/slalom_pole_finder_node.hpp"

#include <cmath>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace slalom_pole_finder
{

SlalomPoleFinderNode::SlalomPoleFinderNode()
: Node("slalom_pole_finder")
{
  fx_ = declare_parameter<double>("fx", 1396.8086675);
  fy_ = declare_parameter<double>("fy", 1396.8086675);
  cx_ = declare_parameter<double>("cx", 960.0);
  cy_ = declare_parameter<double>("cy", 540.0);
  object_height_ = declare_parameter<double>("object_height", 0.938215);

  // Camera optical frame: x right, y down, z forward.
  // Default assumes a forward-facing camera and a drone body frame with
  // x forward, y right, z down (FRD): (x, y, z)_camera -> (z, x, y)_body.
  // Replace these values with the measured camera extrinsics if the mount differs.

  cam_tx_ = declare_parameter<double>("camera.tx", 0.37477);
  cam_ty_ = declare_parameter<double>("camera.ty", 0.0115);
  cam_tz_ = declare_parameter<double>("camera.tz", -0.037895);
  cam_qx_ = declare_parameter<double>("camera.qx", 0.5);
  cam_qy_ = declare_parameter<double>("camera.qy", 0.5);
  cam_qz_ = declare_parameter<double>("camera.qz", 0.5);
  cam_qw_ = declare_parameter<double>("camera.qw", 0.5);
  edge_margin_px_ = declare_parameter<double>("edge_margin_px", 20.0);
  image_width_ = declare_parameter<int>("image_width", 1920);
  image_height_ = declare_parameter<int>("image_height", 1080);
  match_distance_m_ = declare_parameter<double>("match_distance_m", 0.75);

  if (fx_ <= 0.0 || fy_ <= 0.0 || object_height_ <= 0.0 ||
    !std::isfinite(fx_) || !std::isfinite(fy_) ||
    !std::isfinite(cx_) || !std::isfinite(cy_) ||
    !std::isfinite(object_height_))
  {
    throw std::invalid_argument("Camera intrinsics and object_height must be finite and positive");
  }
  if (!std::isfinite(edge_margin_px_) || edge_margin_px_ < 0.0 ||
    image_width_ <= 0 || image_height_ <= 0 ||
    edge_margin_px_ * 2.0 >= image_width_ || edge_margin_px_ * 2.0 >= image_height_ ||
    !std::isfinite(match_distance_m_) || match_distance_m_ <= 0.0)
  {
    throw std::invalid_argument("Image size and edge/tracking parameters are invalid");
  }

  const std::string detections_topic = declare_parameter<std::string>(
    "detections_topic", "/yolo_object_detection/detections");
  const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/nautilus/odom");
  const std::string positions_topic = declare_parameter<std::string>(
    "positions_topic", "/slalom_pole_finder/poles_3d");
  const std::string landmarks_topic = declare_parameter<std::string>(
    "landmarks_topic", "/nautilus/landmarks");

  detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    detections_topic, 10,
    [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
      detectionCallback(msg);
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 20,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      odomCallback(msg);
    });
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(positions_topic, 10);
  landmark_pub_ = create_publisher<vortex_msgs::msg::LandmarkArray>(
    landmarks_topic, 10);

  RCLCPP_INFO(get_logger(), "Estimating 3D poles from '%s' and '%s'",
    detections_topic.c_str(), odom_topic.c_str());
}

void SlalomPoleFinderNode::odomCallback(
  nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  latest_odom_ = msg;
}

bool SlalomPoleFinderNode::touchesImageEdge(
  const vision_msgs::msg::Detection2D & detection) const
{
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
  const geometry_msgs::msg::Point & position,
  std::unordered_set<std::size_t> & matched_tracks)
{
  const double max_distance_squared = match_distance_m_ * match_distance_m_;
  double best_distance_squared = max_distance_squared;
  std::size_t best_index = tracked_poles_.size();

  for (std::size_t index = 0; index < tracked_poles_.size(); ++index) {
    if (matched_tracks.count(index) != 0) {
      continue;
    }
    const auto & previous = tracked_poles_[index].position;
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

  auto & track = tracked_poles_[best_index];
  track.position = position;
  matched_tracks.insert(best_index);
  return track.id;
}

void SlalomPoleFinderNode::detectionCallback(
  vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
{
  // main.cpp uses a single-threaded executor, so the latest odometry pointer is
  // never changed concurrently with this callback.
  if (!latest_odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odometry");
    return;
  }

  const auto & odom = *latest_odom_;
  if (odom.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Odometry has no reference frame");
    return;
  }

  tf2::Quaternion q_base_camera(cam_qx_, cam_qy_, cam_qz_, cam_qw_);
  tf2::Quaternion q_odom_base(
    odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  if (q_base_camera.length2() < 1e-12 || q_odom_base.length2() < 1e-12) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Camera or odometry rotation is invalid");
    return;
  }
  q_base_camera.normalize();
  q_odom_base.normalize();

  // Express the odometry frame's vertical pole axis in camera coordinates.
  // Rotating the vertical unit vector into the body frame automatically
  // removes yaw because yaw rotates around that same vertical axis. Only
  // vehicle tilt (roll/pitch) and the camera mount affect projected length.
  const tf2::Vector3 pole_axis_base = tf2::quatRotate(
    q_odom_base.inverse(), tf2::Vector3(0.0, 0.0, 1.0));
  const tf2::Vector3 pole_axis_camera = tf2::quatRotate(
    q_base_camera.inverse(), pole_axis_base);

  const tf2::Vector3 t_base_camera(cam_tx_, cam_ty_, cam_tz_);
  const tf2::Vector3 t_odom_base(
    odom.pose.pose.position.x, odom.pose.pose.position.y,
    odom.pose.pose.position.z);

  geometry_msgs::msg::PoseArray output;
  output.header.stamp = msg->header.stamp;
  output.header.frame_id = odom.header.frame_id;
  vortex_msgs::msg::LandmarkArray landmark_output;
  landmark_output.header = output.header;

  if (tracking_frame_ != odom.header.frame_id) {
    tracked_poles_.clear();
    tracking_frame_ = odom.header.frame_id;
  }
  std::unordered_set<std::size_t> matched_tracks;

  for (const auto & detection : msg->detections) {
    const double u = detection.bbox.center.position.x;
    const double v = detection.bbox.center.position.y;
    const double w_px = detection.bbox.size_x;
    const double h_px = detection.bbox.size_y;
    if (!std::isfinite(u) || !std::isfinite(v) ||
      !std::isfinite(w_px) || !std::isfinite(h_px) ||
      w_px <= 0.0 || h_px <= 1.0 || touchesImageEdge(detection))
    {
      continue;
    }

    if (detection.results.empty()) {
      continue;
    }

    std::uint16_t landmark_subtype;
    const std::string & class_id = detection.results.front().hypothesis.class_id;
    if (class_id == "0") {
      landmark_subtype = vortex_msgs::msg::LandmarkSubtype::SLALOM_PIPE_RED;
    } else if (class_id == "1") {
      landmark_subtype = vortex_msgs::msg::LandmarkSubtype::SLALOM_PIPE_WHITE;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring unsupported YOLO class ID: %s", class_id.c_str());
      continue;
    }

    // The diagonal approximates the projected length of a narrow pole even
    // when roll rotates it in the image. The projection scale below corrects
    // for pitch and for an off-axis pole.
    const double observed_length_px = std::hypot(w_px, h_px);
    const double normalized_x = (u - cx_) / fx_;
    const double normalized_y = (v - cy_) / fy_;
    const double projected_x = pole_axis_camera.x() -
      normalized_x * pole_axis_camera.z();
    const double projected_y = pole_axis_camera.y() -
      normalized_y * pole_axis_camera.z();
    const double projection_scale_px = std::hypot(
      fx_ * projected_x, fy_ * projected_y);
    if (!std::isfinite(projection_scale_px) || projection_scale_px < 1e-9) {
      continue;
    }

    // Solve the perspective projection of a pole centered on the detection
    // ray. The quadratic term matters when the pole axis has a component in
    // the camera-forward direction (for example, while the drone is pitched).
    const double half_depth_extent =
      0.5 * object_height_ * pole_axis_camera.z();
    const double scaled_height = object_height_ * projection_scale_px;
    const double discriminant =
      scaled_height * scaled_height +
      4.0 * observed_length_px * observed_length_px *
      half_depth_extent * half_depth_extent;
    const double depth =
      (scaled_height + std::sqrt(discriminant)) /
      (2.0 * observed_length_px);
    if (!std::isfinite(depth) || depth <= std::abs(half_depth_extent)) {
      continue;
    }

    const tf2::Vector3 p_camera(
      normalized_x * depth, normalized_y * depth, depth);
    // With the default optical->FRD rotation, p_base.z() is the object's
    // downward offset from the drone. The odometry rotation accounts for
    // vehicle roll and pitch before its position is added.
    const tf2::Vector3 p_base =
      tf2::quatRotate(q_base_camera, p_camera) + t_base_camera;
    const tf2::Vector3 p_odom =
      tf2::quatRotate(q_odom_base, p_base) + t_odom_base;

    geometry_msgs::msg::Pose pose;
    pose.position.x = p_odom.x();
    pose.position.y = p_odom.y();
    pose.position.z = p_odom.z();  // Down-positive when odometry is NED.
    pose.orientation.w = 1.0;  // The bounding box does not provide orientation.
    output.poses.push_back(pose);

    vortex_msgs::msg::Landmark landmark;
    landmark.header = landmark_output.header;
    landmark.id = assignId(pose.position, matched_tracks);
    landmark.type.value = vortex_msgs::msg::LandmarkType::SLALOM_PIPE;
    landmark.subtype.value = landmark_subtype;
    landmark.pose.pose = pose;
    landmark.pose.covariance.fill(1.0);
    landmark_output.landmarks.push_back(landmark);
  }

  pose_pub_->publish(output);
  landmark_pub_->publish(landmark_output);
}

}  // namespace slalom_pole_finder
