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
    clamp_rotation_ = declare_parameter<bool>("clamp_rotation");

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
        depth_color_extrinsic_.R = Eigen::Matrix3f::Identity();
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
        extrinsic_timer_ =
            create_wall_timer(std::chrono::milliseconds(500),
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

    try_activate_detector();
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
    detector_->set_clamp_rotation(clamp_rotation_);
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

    // Slop set to 100 ms to accommodate the detection pipeline latency
    // (~66 ms observed between depth and detection timestamps).
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(20), depth_sub_, det_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.15));
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

std::vector<std::pair<float, BoundingBox>> ValvePoseNode::collect_scored_boxes(
    const vision_msgs::msg::Detection2DArray& det) const {
    std::vector<std::pair<float, BoundingBox>> boxes;
    boxes.reserve(det.detections.size());
    for (const auto& d : det.detections) {
        const float score =
            d.results.empty()
                ? static_cast<float>(d.bbox.size_x * d.bbox.size_y)
                : static_cast<float>(d.results[0].hypothesis.score);
        boxes.emplace_back(score, to_bbox(d.bbox));
    }
    return boxes;
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
    const std::vector<BoundingBox>& boxes,
    const std::vector<Pose>& poses,
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

    if (depth_colormap_pub_) {
        cv::Mat colormap = build_depth_colormap(depth);
        for (size_t i = 0; i < boxes.size(); ++i) {
            const auto& box = boxes[i];

            // poses[i] is in the depth camera frame.
            // project_color_pixel_to_depth needs Z in the color camera frame,
            // so transform the 3-D point first.
            const Eigen::Vector3f P_depth(static_cast<float>(poses[i].x),
                                          static_cast<float>(poses[i].y),
                                          static_cast<float>(poses[i].z));
            const Eigen::Vector3f P_color =
                depth_color_extrinsic_.R * P_depth + depth_color_extrinsic_.t;
            const float Z_color = P_color.z();
            if (Z_color <= 0.0f)
                continue;

            const float angle_deg =
                box.theta * 180.0f / static_cast<float>(M_PI);
            cv::RotatedRect rrect(cv::Point2f(box.center_x, box.center_y),
                                  cv::Size2f(box.size_x, box.size_y),
                                  angle_deg);
            cv::Point2f corners[4];
            rrect.points(corners);
            for (auto& c : corners)
                c = project_color_pixel_to_depth(c.x, c.y, Z_color,
                                                 color_props_, depth_props_,
                                                 depth_color_extrinsic_);
            for (int j = 0; j < 4; ++j)
                cv::line(colormap, corners[j], corners[(j + 1) % 4],
                         cv::Scalar(0, 255, 0), 2);
        }
        depth_colormap_pub_->publish(
            *cv_bridge::CvImage(depth->header, "bgr8", colormap).toImageMsg());
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

// Main synchronized callback: runs NMS, estimates poses, and publishes all
// outputs.
void ValvePoseNode::sync_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det) {
    if (!depth || !det || !detector_)
        return;

    const auto scored_boxes = collect_scored_boxes(*det);
    const std::vector<size_t> kept =
        filter_duplicate_detections(scored_boxes, iou_duplicate_threshold_);

    std_msgs::msg::Header pose_header = depth->header;
    pose_header.frame_id = output_frame_id_;

    if (det->detections.empty()) {
        publish_empty_results(pose_header);
        if (debug_visualize_ && depth_colormap_pub_)
            depth_colormap_pub_->publish(
                *cv_bridge::CvImage(depth->header, "bgr8",
                                    build_depth_colormap(depth))
                     .toImageMsg());
        return;
    }

    const cv::Mat depth_img = decode_depth_to_float(depth);
    const DetectorMode mode =
        debug_visualize_ ? DetectorMode::debug : DetectorMode::standard;

    std::vector<BoundingBox> raw_boxes;
    std::vector<Pose> poses;
    pcl::PointCloud<pcl::PointXYZ> ann_dbg, pln_dbg;

    for (size_t idx : kept) {
        const BoundingBox& yolo_box = scored_boxes[idx].second;
        // YOLO outputs in 640×640 letterbox space — convert to color image
        // space, then optionally correct for lens distortion.
        BoundingBox color_box = detector_->letterbox_to_image_coords(yolo_box);
        if (undistort_detections_)
            color_box = undistort_bbox(color_box, color_props_.intr);

        const auto result =
            detector_->compute_pose_from_depth(depth_img, color_box, mode);
        if (!result.valid)
            continue;

        // Keep raw_boxes and poses aligned: only push when pose succeeded.
        raw_boxes.push_back(color_box);
        poses.push_back(result.pose);
        if (mode == DetectorMode::debug) {
            if (result.annulus_cloud)
                ann_dbg += *result.annulus_cloud;
            if (result.plane_cloud)
                pln_dbg += *result.plane_cloud;
        }
    }

    if (debug_visualize_)
        publish_debug(depth, raw_boxes, poses, ann_dbg, pln_dbg);

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
            pt1.x = p1.x(); pt1.y = p1.y(); pt1.z = p1.z();
            pt2.x = p2.x(); pt2.y = p2.y(); pt2.z = p2.z();
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
            s1.color.r = 1.0f; s1.color.g = 0.0f; s1.color.b = 0.0f;
            s1.color.a = 1.0f;

            visualization_msgs::msg::Marker s2 = s1;
            s2.id = static_cast<int>(i * 2 + 1);
            s2.pose.position = pt2;
            s2.color.r = 0.0f; s2.color.g = 0.0f; s2.color.b = 1.0f;

            marker_array.markers.push_back(m);
            marker_array.markers.push_back(s1);
            marker_array.markers.push_back(s2);
        }
        handle_marker_pub_->publish(marker_array);
    }
}

}  // namespace valve_detection

RCLCPP_COMPONENTS_REGISTER_NODE(valve_detection::ValvePoseNode)
