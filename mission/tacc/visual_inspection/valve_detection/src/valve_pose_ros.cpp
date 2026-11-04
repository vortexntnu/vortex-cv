// valve_pose_ros.cpp
// ROS node: subscribes to depth and detections; publishes valve poses
// and visualizations.
#include "valve_detection_ros/valve_pose_ros.hpp"
#include "valve_detection/depth_image_processing.hpp"
#include "valve_detection_ros/ros_utils.hpp"

#include <tf2/exceptions.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include <vortex/utils/ros/qos_profiles.hpp>

namespace valve_detection {

using std::placeholders::_1;
using std::placeholders::_2;

ValvePoseNode::ValvePoseNode(const rclcpp::NodeOptions& options)
    : Node("valve_pose_node", options) {
    declare_params();
    init_subscriptions();
}

void ValvePoseNode::declare_params() {
    debug_visualize_ = declare_parameter<bool>("debug_visualize");
    iou_duplicate_threshold_ = static_cast<float>(
        declare_parameter<double>("iou_duplicate_threshold"));
    score_threshold_ =
        static_cast<float>(declare_parameter<double>("score_threshold", 0.6));

    const std::string frame_base =
        declare_parameter<std::string>("output_frame_id");
    if (frame_base.empty()) {
        throw std::runtime_error("output_frame_id must not be empty");
    }
    const std::string drone =
        declare_parameter<std::string>("drone", "nautilus");
    output_frame_id_ = drone + "/" + frame_base;

    // Estimator config params
    yolo_w_ = declare_parameter<int>("yolo_img_width");
    yolo_h_ = declare_parameter<int>("yolo_img_height");
    annulus_ratio_ = declare_parameter<float>("annulus_radius_ratio");
    ransac_thresh_ = declare_parameter<float>("plane_ransac_threshold");
    ransac_iters_ = declare_parameter<int>("plane_ransac_max_iterations");
    handle_offset_ = declare_parameter<float>("valve_handle_offset");
    undistort_detections_ = declare_parameter<bool>("undistort_detections");

    yaw_closed_reference_rad_ = static_cast<float>(
        declare_parameter<double>("yaw_closed_reference_deg", 0.0) * M_PI /
        180.0);
    yaw_invert_ = declare_parameter<bool>("yaw_invert", false);

    // TF frame IDs for the depth-to-color extrinsic lookup.
    const std::string depth_frame_base =
        declare_parameter<std::string>("depth_frame_id");
    const std::string color_frame_base =
        declare_parameter<std::string>("color_frame_id");
    depth_frame_id_ = drone + "/" + depth_frame_base;
    color_frame_id_ = drone + "/" + color_frame_base;

    use_hardcoded_extrinsic_ =
        declare_parameter<bool>("use_hardcoded_extrinsic");

    if (use_hardcoded_extrinsic_) {
        const double tx = declare_parameter<double>("extrinsic_tx");
        const double ty = declare_parameter<double>("extrinsic_ty");
        const double tz = declare_parameter<double>("extrinsic_tz");
        const auto R_vec = declare_parameter<std::vector<double>>(
            "extrinsic_R",
            std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        if (R_vec.size() != 9) {
            throw std::runtime_error(
                "extrinsic_R must have exactly 9 elements (row-major 3x3)");
        }
        Eigen::Matrix3f R;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R(i, j) = static_cast<float>(R_vec[i * 3 + j]);
            }
        }
        depth_color_extrinsic_.R = R;
        depth_color_extrinsic_.t =
            Eigen::Vector3f(static_cast<float>(tx), static_cast<float>(ty),
                            static_cast<float>(tz));
        extrinsic_ready_ = true;
        RCLCPP_INFO(get_logger(),
                    "Using hardcoded extrinsic (t=[%.4f, %.4f, %.4f])", tx, ty,
                    tz);
    } else {
        // TF2 buffer and listener for extrinsic lookup.
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Periodically attempt to look up the static transform until it
        // arrives.
        extrinsic_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ValvePoseNode::lookup_extrinsic, this));
    }

    if (debug_visualize_) {
        const auto pose_topic =
            declare_parameter<std::string>("debug.valve_poses_pub_topic");
        const auto depth_cloud_topic =
            declare_parameter<std::string>("debug.depth_cloud_pub_topic");
        const auto depth_color_topic =
            declare_parameter<std::string>("debug.depth_colormap_pub_topic");
        const auto ann_topic =
            declare_parameter<std::string>("debug.annulus_pub_topic");
        const auto pln_topic =
            declare_parameter<std::string>("debug.plane_pub_topic");
        depth_colormap_vmin_ = static_cast<float>(
            declare_parameter<double>("debug.depth_colormap_value_min"));
        depth_colormap_vmax_ = static_cast<float>(
            declare_parameter<double>("debug.depth_colormap_value_max"));

        const auto sensor_qos =
            vortex::utils::qos_profiles::reliable_profile(10);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(pose_topic,
                                                                    sensor_qos);
        depth_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            depth_cloud_topic, sensor_qos);
        depth_colormap_pub_ = create_publisher<sensor_msgs::msg::Image>(
            depth_color_topic, sensor_qos);
        annulus_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            ann_topic, sensor_qos);
        plane_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            pln_topic, sensor_qos);
        handle_marker_pub_ =
            create_publisher<visualization_msgs::msg::MarkerArray>(
                "/valve_handle_markers", sensor_qos);
    }

    const auto lm_topic = declare_parameter<std::string>("landmarks_pub_topic");
    landmark_pub_ = create_publisher<vortex_msgs::msg::LandmarkArray>(
        lm_topic, vortex::utils::qos_profiles::reliable_profile(10));

    enable_annotated_image_ =
        declare_parameter<bool>("enable_annotated_image", false);
    if (enable_annotated_image_) {
        const auto annotated_topic = declare_parameter<std::string>(
            "annotated_image_pub_topic",
            "/valve_detection/annotated_image_with_theta");
        const auto color_image_topic =
            declare_parameter<std::string>("color_image_sub_topic");
        const auto theta_classes = declare_parameter<std::vector<std::string>>(
            "annotated_image_theta_classes", std::vector<std::string>{"0"});
        annotated_image_theta_classes_.insert(theta_classes.begin(),
                                              theta_classes.end());

        const auto sensor_qos =
            vortex::utils::qos_profiles::reliable_profile(10);
        annotated_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            annotated_topic, sensor_qos);
        color_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            color_image_topic, rclcpp::QoS(1).best_effort(),
            std::bind(&ValvePoseNode::color_image_cb, this, _1));
    }

    try_activate_detector();
}

void ValvePoseNode::color_image_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    latest_color_image_ = msg;
}

float ValvePoseNode::fold_obb_theta(float size_x,
                                    float size_y,
                                    float theta) const {
    float w = size_x;
    float h = size_y;
    float t = theta;
    if (h > w) {
        std::swap(w, h);
        t += static_cast<float>(M_PI / 2.0);
    }
    const float pi = static_cast<float>(M_PI);
    float raw = std::fmod(t, pi);
    if (raw < 0.0f)
        raw += pi;
    const float sign = yaw_invert_ ? -1.0f : 1.0f;
    float diff = std::fmod(sign * (raw - yaw_closed_reference_rad_), pi);
    if (diff < 0.0f)
        diff += pi;
    return (diff <= pi / 2.0f) ? diff : (pi - diff);
}

void ValvePoseNode::publish_annotated_image(
    const std_msgs::msg::Header& header,
    const std::vector<std::pair<float, BoundingBox>>& scored_valves,
    const std::vector<std::pair<float, BoundingBox>>& scored_handles) {
    if (!annotated_image_pub_ || !latest_color_image_)
        return;

    cv::Mat image;
    try {
        cv_bridge::CvImageConstPtr cv_img =
            cv_bridge::toCvCopy(latest_color_image_, "bgr8");
        image = cv_img->image;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "annotated_image cv_bridge failed: %s", e.what());
        return;
    }

    auto draw = [&](const BoundingBox& raw_box, const std::string& cls,
                    const cv::Scalar& color) {
        BoundingBox b = detections_letterboxed_
                            ? detector_->letterbox_to_image_coords(raw_box)
                            : raw_box;
        const float angle_deg = b.theta * 180.0f / static_cast<float>(M_PI);
        cv::RotatedRect rrect(cv::Point2f(b.center_x, b.center_y),
                              cv::Size2f(b.size_x, b.size_y), angle_deg);
        cv::Point2f corners[4];
        rrect.points(corners);
        for (int j = 0; j < 4; ++j)
            cv::line(image, corners[j], corners[(j + 1) % 4], color, 2);

        if (annotated_image_theta_classes_.count(cls)) {
            const float folded_deg =
                fold_obb_theta(b.size_x, b.size_y, b.theta) * 180.0f /
                static_cast<float>(M_PI);
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.1f deg", folded_deg);
            cv::putText(image, buf,
                        cv::Point(static_cast<int>(b.center_x),
                                  static_cast<int>(b.center_y) - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv::LINE_AA);
        }
    };

    for (const auto& [score, box] : scored_valves)
        draw(box, "1", cv::Scalar(0, 255, 0));
    for (const auto& [score, box] : scored_handles)
        draw(box, "0", cv::Scalar(0, 165, 255));

    auto out_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    annotated_image_pub_->publish(*out_msg);
}

void ValvePoseNode::lookup_extrinsic() {
    if (extrinsic_ready_)
        return;

    try {
        const auto tf = tf_buffer_->lookupTransform(
            color_frame_id_, depth_frame_id_, tf2::TimePointZero);
        const auto& t = tf.transform.translation;
        const auto& q = tf.transform.rotation;

        const Eigen::Quaternionf quat(
            static_cast<float>(q.w), static_cast<float>(q.x),
            static_cast<float>(q.y), static_cast<float>(q.z));
        depth_color_extrinsic_.R = quat.toRotationMatrix();
        depth_color_extrinsic_.t =
            Eigen::Vector3f(static_cast<float>(t.x), static_cast<float>(t.y),
                            static_cast<float>(t.z));

        extrinsic_ready_ = true;
        extrinsic_timer_->cancel();
        RCLCPP_INFO(get_logger(),
                    "Depth-to-color extrinsic loaded from TF (%s -> %s, "
                    "t=[%.4f, %.4f, %.4f])",
                    depth_frame_id_.c_str(), color_frame_id_.c_str(), t.x, t.y,
                    t.z);

        if (detector_) {
            detector_->set_depth_color_extrinsic(depth_color_extrinsic_);
        } else {
            try_activate_detector();
        }
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Waiting for TF: %s -> %s: %s",
            depth_frame_id_.c_str(), color_frame_id_.c_str(), ex.what());
    }
}

void ValvePoseNode::try_activate_detector() {
    if (!color_props_ready_ || !depth_props_ready_ || !extrinsic_ready_ ||
        detector_)
        return;

    detector_ = std::make_unique<PoseEstimator>(yolo_w_, yolo_h_,
                                                annulus_ratio_, ransac_thresh_,
                                                ransac_iters_, handle_offset_);
    detector_->set_color_image_properties(color_props_);
    detector_->set_depth_image_properties(depth_props_);
    detector_->set_depth_color_extrinsic(depth_color_extrinsic_);
    detector_->compute_letterbox_transform();
    RCLCPP_INFO(get_logger(), "Detector initialised");
}

void ValvePoseNode::init_subscriptions() {
    const auto depth_topic =
        declare_parameter<std::string>("depth_image_sub_topic");
    const auto det_topic =
        declare_parameter<std::string>("detections_sub_topic");
    const auto depth_info_topic =
        declare_parameter<std::string>("depth_image_info_topic");
    const auto color_info_topic =
        declare_parameter<std::string>("color_image_info_topic");

    const auto info_qos = vortex::utils::qos_profiles::sensor_data_profile(1);

    color_cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        color_info_topic, info_qos,
        std::bind(&ValvePoseNode::color_camera_info_cb, this, _1));
    depth_cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_info_topic, info_qos,
        std::bind(&ValvePoseNode::depth_camera_info_cb, this, _1));

    const auto data_qos = vortex::utils::qos_profiles::sensor_data_profile(10);
    depth_sub_.subscribe(this, depth_topic, data_qos.get_rmw_qos_profile());
    det_sub_.subscribe(this, det_topic, data_qos.get_rmw_qos_profile());

    // Slop accommodates detection pipeline latency and bag-replay timestamp
    // jitter between depth and detection streams.
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(50), depth_sub_, det_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.3));
    sync_->registerCallback(std::bind(&ValvePoseNode::sync_cb, this, _1, _2));
}

// One-shot callback that overrides color intrinsics and distortion from the
// camera_info topic.
void ValvePoseNode::color_camera_info_cb(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    color_props_.intr.fx = msg->k[0];
    color_props_.intr.fy = msg->k[4];
    color_props_.intr.cx = msg->k[2];
    color_props_.intr.cy = msg->k[5];
    for (size_t i = 0; i < 5 && i < msg->d.size(); ++i)
        color_props_.intr.dist[i] = msg->d[i];
    color_props_.dim.width = static_cast<int>(msg->width);
    color_props_.dim.height = static_cast<int>(msg->height);
    color_props_ready_ = true;
    color_cam_info_sub_.reset();
    RCLCPP_INFO(get_logger(), "Color camera_info received (fx=%.2f fy=%.2f)",
                color_props_.intr.fx, color_props_.intr.fy);
    if (detector_) {
        detector_->set_color_image_properties(color_props_);
        detector_->compute_letterbox_transform();
    } else {
        try_activate_detector();
    }
}

// One-shot callback that overrides depth intrinsics from the camera_info topic.
void ValvePoseNode::depth_camera_info_cb(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    depth_props_.intr.fx = msg->k[0];
    depth_props_.intr.fy = msg->k[4];
    depth_props_.intr.cx = msg->k[2];
    depth_props_.intr.cy = msg->k[5];
    depth_props_.dim.width = static_cast<int>(msg->width);
    depth_props_.dim.height = static_cast<int>(msg->height);
    depth_props_ready_ = true;
    depth_cam_info_sub_.reset();
    RCLCPP_INFO(get_logger(), "Depth camera_info received (fx=%.1f fy=%.1f)",
                depth_props_.intr.fx, depth_props_.intr.fy);
    if (detector_) {
        detector_->set_depth_image_properties(depth_props_);
    } else {
        try_activate_detector();
    }
}

ValvePoseNode::SplitDetections ValvePoseNode::split_scored_boxes(
    const vision_msgs::msg::Detection2DArray& det) const {
    SplitDetections out;
    out.valves.reserve(det.detections.size());
    out.handles.reserve(det.detections.size());
    for (const auto& d : det.detections) {
        if (d.results.empty())
            continue;
        const float score = static_cast<float>(d.results[0].hypothesis.score);
        if (score < score_threshold_)
            continue;
        const std::string& cls = d.results[0].hypothesis.class_id;
        if (cls == "1")
            out.valves.emplace_back(score, to_bbox(d.bbox));
        else if (cls == "0")
            out.handles.emplace_back(score, to_bbox(d.bbox));
    }
    return out;
}

// Axis-aligned check against the valve bbox's OBB (rotated by theta around
// its center).  `point` and `box` must share the same coordinate space.
static bool point_in_obb(float px, float py, const BoundingBox& box) {
    const float dx = px - box.center_x;
    const float dy = py - box.center_y;
    const float c = std::cos(-box.theta);
    const float s = std::sin(-box.theta);
    const float lx = c * dx - s * dy;
    const float ly = s * dx + c * dy;
    return std::abs(lx) <= box.size_x * 0.5f &&
           std::abs(ly) <= box.size_y * 0.5f;
}

void ValvePoseNode::publish_empty_results(
    const std_msgs::msg::Header& header) const {
    if (debug_visualize_ && pose_pub_)
        pose_pub_->publish(make_pose_array({}, header));
    landmark_pub_->publish(make_landmark_array({}, header));
}

cv::Mat ValvePoseNode::build_depth_colormap(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth) const {
    cv_bridge::CvImageConstPtr cv_raw = cv_bridge::toCvShare(depth, "16UC1");
    cv::Mat depth_f;
    cv_raw->image.convertTo(depth_f, CV_32FC1);
    const float scale = 255.0f / (depth_colormap_vmax_ - depth_colormap_vmin_);
    cv::Mat depth_8u;
    depth_f.convertTo(depth_8u, CV_8UC1, scale, -depth_colormap_vmin_ * scale);
    cv::Mat colormap;
    cv::applyColorMap(depth_8u, colormap, cv::COLORMAP_TURBO);
    return colormap;
}

void ValvePoseNode::publish_debug(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth,
    const pcl::PointCloud<pcl::PointXYZ>& ann_cloud,
    const pcl::PointCloud<pcl::PointXYZ>& pln_cloud) const {
    std_msgs::msg::Header pcl_header = depth->header;
    pcl_header.frame_id = output_frame_id_;

    if (depth_cloud_pub_ && depth_cloud_pub_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(ann_cloud, msg);
        msg.header = pcl_header;
        depth_cloud_pub_->publish(msg);
    }

    if (annulus_pub_ && plane_pub_) {
        sensor_msgs::msg::PointCloud2 ann_msg, pln_msg;
        pcl::toROSMsg(ann_cloud, ann_msg);
        pcl::toROSMsg(pln_cloud, pln_msg);
        ann_msg.header = pcl_header;
        pln_msg.header = pcl_header;
        annulus_pub_->publish(ann_msg);
        plane_pub_->publish(pln_msg);
    }
}

// Publishes the depth colormap with every NMS-filtered detection drawn:
// valves green, handles orange. Z is sampled from the depth image at each
// box center (color->depth via fx ratio), with a 1 m fallback on invalid
// depth. Runs every sync_cb tick — does not depend on pose-fit success.
void ValvePoseNode::publish_box_colormap(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth,
    const cv::Mat& depth_img,
    const std::vector<BoundingBox>& valve_boxes,
    const std::vector<BoundingBox>& handle_boxes) const {
    if (!depth_colormap_pub_)
        return;

    cv::Mat colormap = build_depth_colormap(depth);

    const float scale =
        (color_props_.intr.fx > 0.0)
            ? static_cast<float>(depth_props_.intr.fx / color_props_.intr.fx)
            : 1.0f;

    auto draw_box = [&](const BoundingBox& box, const cv::Scalar& color) {
        const int u_d = std::clamp(static_cast<int>(box.center_x * scale), 0,
                                   depth_img.cols - 1);
        const int v_d = std::clamp(static_cast<int>(box.center_y * scale), 0,
                                   depth_img.rows - 1);
        const float Z = depth_img.at<float>(v_d, u_d);
        const float Z_color = (std::isfinite(Z) && Z > 0.0f) ? Z : 1.0f;

        const float angle_deg = box.theta * 180.0f / static_cast<float>(M_PI);
        cv::RotatedRect rrect(cv::Point2f(box.center_x, box.center_y),
                              cv::Size2f(box.size_x, box.size_y), angle_deg);
        cv::Point2f corners[4];
        rrect.points(corners);
        for (auto& c : corners)
            c = project_color_pixel_to_depth(c.x, c.y, Z_color, color_props_,
                                             depth_props_,
                                             depth_color_extrinsic_);
        for (int j = 0; j < 4; ++j)
            cv::line(colormap, corners[j], corners[(j + 1) % 4], color, 2);
    };

    for (const auto& v : valve_boxes)
        draw_box(v, cv::Scalar(0, 255, 0));  // green
    for (const auto& h : handle_boxes)
        draw_box(h, cv::Scalar(0, 165, 255));  // orange

    depth_colormap_pub_->publish(
        *cv_bridge::CvImage(depth->header, "bgr8", colormap).toImageMsg());
}

// Main synchronized callback: runs NMS, estimates poses, and publishes all
// outputs.
void ValvePoseNode::sync_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det) {
    if (!depth || !det || !detector_)
        return;

    const SplitDetections split = split_scored_boxes(*det);

    std_msgs::msg::Header pose_header = depth->header;
    pose_header.frame_id = output_frame_id_;

    // Per-class NMS: valves against valves, handles against handles.
    const std::vector<size_t> kept_valves =
        filter_duplicate_detections(split.valves, iou_duplicate_threshold_);
    const std::vector<size_t> kept_handles =
        filter_duplicate_detections(split.handles, iou_duplicate_threshold_);

    // Transform kept boxes into color image space once so pairing happens
    // in the same coordinate frame as compute_pose_from_depth consumes.
    auto to_color_space = [&](const BoundingBox& raw_box) {
        BoundingBox b = detections_letterboxed_
                            ? detector_->letterbox_to_image_coords(raw_box)
                            : raw_box;
        if (undistort_detections_)
            b = undistort_bbox(b, color_props_.intr);
        return b;
    };

    std::vector<BoundingBox> valve_boxes;
    valve_boxes.reserve(kept_valves.size());
    for (size_t idx : kept_valves)
        valve_boxes.push_back(to_color_space(split.valves[idx].second));

    std::vector<BoundingBox> handle_boxes;
    handle_boxes.reserve(kept_handles.size());
    for (size_t idx : kept_handles)
        handle_boxes.push_back(to_color_space(split.handles[idx].second));

    // Decode depth once. Used for both the always-on colormap overlay and
    // the pose-fit pipeline below.
    const cv::Mat depth_img = decode_depth_to_float(depth);

    // Always publish the colormap with every NMS-filtered detection drawn,
    // independent of pairing or pose-fit success.
    if (debug_visualize_)
        publish_box_colormap(depth, depth_img, valve_boxes, handle_boxes);

    // Always publish the annotated color image (raw OBBs + folded theta text
    // for configured classes), independent of pairing/pose-fit success.
    if (enable_annotated_image_) {
        std::vector<std::pair<float, BoundingBox>> scored_valves;
        std::vector<std::pair<float, BoundingBox>> scored_handles;
        scored_valves.reserve(kept_valves.size());
        for (size_t idx : kept_valves)
            scored_valves.push_back(split.valves[idx]);
        scored_handles.reserve(kept_handles.size());
        for (size_t idx : kept_handles)
            scored_handles.push_back(split.handles[idx]);
        std_msgs::msg::Header img_header = depth->header;
        if (latest_color_image_)
            img_header = latest_color_image_->header;
        publish_annotated_image(img_header, scored_valves, scored_handles);
    }

    if (valve_boxes.empty()) {
        publish_empty_results(pose_header);
        return;
    }

    // Pair each valve with at most one handle (handle center inside the
    // valve OBB; ties broken by smaller center-to-center distance).
    // Unpaired valves are dropped.
    std::vector<std::pair<const BoundingBox*, const BoundingBox*>> pairs;
    std::vector<bool> handle_used(handle_boxes.size(), false);
    pairs.reserve(valve_boxes.size());
    for (const auto& v : valve_boxes) {
        int best = -1;
        float best_d2 = std::numeric_limits<float>::max();
        for (size_t j = 0; j < handle_boxes.size(); ++j) {
            if (handle_used[j])
                continue;
            const auto& h = handle_boxes[j];
            if (!point_in_obb(h.center_x, h.center_y, v))
                continue;
            const float dx = h.center_x - v.center_x;
            const float dy = h.center_y - v.center_y;
            const float d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best = static_cast<int>(j);
            }
        }
        if (best < 0)
            continue;
        handle_used[best] = true;
        pairs.emplace_back(&v, &handle_boxes[best]);
    }

    if (pairs.empty()) {
        publish_empty_results(pose_header);
        return;
    }

    const DetectorMode mode =
        debug_visualize_ ? DetectorMode::debug : DetectorMode::standard;

    std::vector<Pose> poses;
    pcl::PointCloud<pcl::PointXYZ> ann_dbg, pln_dbg;

    for (const auto& [valve_ptr, handle_ptr] : pairs) {
        const BoundingBox& valve_box = *valve_ptr;
        const float handle_angle = fold_obb_theta(
            handle_ptr->size_x, handle_ptr->size_y, handle_ptr->theta);

        const auto result = detector_->compute_pose_from_depth(
            depth_img, valve_box, handle_angle, mode);
        if (!result.valid)
            continue;

        poses.push_back(result.pose);
        if (mode == DetectorMode::debug) {
            if (result.annulus_cloud)
                ann_dbg += *result.annulus_cloud;
            if (result.plane_cloud)
                pln_dbg += *result.plane_cloud;
        }
    }

    if (debug_visualize_)
        publish_debug(depth, ann_dbg, pln_dbg);

    if (debug_visualize_ && pose_pub_)
        pose_pub_->publish(make_pose_array(poses, pose_header));
    landmark_pub_->publish(make_landmark_array(poses, pose_header));

    // Publish handle direction line markers for Foxglove visualization.
    if (debug_visualize_ && handle_marker_pub_) {
        visualization_msgs::msg::MarkerArray marker_array;
        // First, add a DELETE_ALL marker to clear old markers.
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        const float line_half_len = 0.05f;  // 5 cm each side
        for (size_t i = 0; i < poses.size(); ++i) {
            const auto& p = poses[i];
            Eigen::Quaterniond q(p.qw, p.qx, p.qy, p.qz);
            Eigen::Matrix3d R = q.normalized().toRotationMatrix();
            Eigen::Vector3d pos(p.x, p.y, p.z);
            Eigen::Vector3d x_axis = R.col(0);  // handle direction

            Eigen::Vector3d p1 = pos - line_half_len * x_axis;
            Eigen::Vector3d p2 = pos + line_half_len * x_axis;

            visualization_msgs::msg::Marker m;
            m.header = pose_header;
            m.ns = "valve_handle";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.005;  // line width 5 mm
            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;

            geometry_msgs::msg::Point pt1, pt2;
            pt1.x = p1.x();
            pt1.y = p1.y();
            pt1.z = p1.z();
            pt2.x = p2.x();
            pt2.y = p2.y();
            pt2.z = p2.z();
            m.points.push_back(pt1);
            m.points.push_back(pt2);

            // Sphere at each endpoint
            visualization_msgs::msg::Marker s1;
            s1.header = pose_header;
            s1.ns = "valve_handle_endpoints";
            s1.id = static_cast<int>(i * 2);
            s1.type = visualization_msgs::msg::Marker::SPHERE;
            s1.action = visualization_msgs::msg::Marker::ADD;
            s1.pose.position = pt1;
            s1.pose.orientation.w = 1.0;
            s1.scale.x = s1.scale.y = s1.scale.z = 0.01;
            s1.color.r = 1.0f;
            s1.color.g = 0.0f;
            s1.color.b = 0.0f;
            s1.color.a = 1.0f;

            visualization_msgs::msg::Marker s2 = s1;
            s2.id = static_cast<int>(i * 2 + 1);
            s2.pose.position = pt2;
            s2.color.r = 0.0f;
            s2.color.g = 0.0f;
            s2.color.b = 1.0f;

            marker_array.markers.push_back(m);
            marker_array.markers.push_back(s1);
            marker_array.markers.push_back(s2);
        }
        handle_marker_pub_->publish(marker_array);
    }
}

}  // namespace valve_detection

RCLCPP_COMPONENTS_REGISTER_NODE(valve_detection::ValvePoseNode)
