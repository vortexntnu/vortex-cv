#include <pipeline_line_fitting/pipeline_line_fitting_ros.hpp>

#include <vortex_msgs/msg/line_segment2_d.hpp>

using std::placeholders::_1;

HoughParams PipelineLineFittingNode::fetchParams() {
    this->declare_parameter("minTurnAngle", 1.5);
    this->declare_parameter("size", 200);
    this->declare_parameter("morph_close_size", 10);
    this->declare_parameter("dist_thresh", 0.2);
    this->declare_parameter("min_skeleton_component_size", 0);
    this->declare_parameter("hough_threshold", 30);
    HoughParams params;
    params.minTurnAngle = this->get_parameter("minTurnAngle").as_double();
    params.size = this->get_parameter("size").as_int();
    params.morph_close_size = this->get_parameter("morph_close_size").as_int();
    params.dist_thresh = this->get_parameter("dist_thresh").as_double();
    params.min_skeleton_component_size =
        this->get_parameter("min_skeleton_component_size").as_int();
    params.hough_threshold = this->get_parameter("hough_threshold").as_int();
    return params;
}

PipelineLineFittingNode::PipelineLineFittingNode(
    const rclcpp::NodeOptions& options)
    : Node("pipeline_line_fitting_node", options) {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                           .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    auto imageSubTopic = this->declare_parameter<std::string>(
        "image_sub_topic", "/cam_down/image_color");
    auto image_visualization_pub_topic = this->declare_parameter<std::string>(
        "image_visualization_pub_topic", "/linefitting/visualization");
    auto lines_pub_topic = this->declare_parameter<std::string>(
        "lines_pub_topic", "/irls_line/lines");

    imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        imageSubTopic, qos_profile,
        std::bind(&PipelineLineFittingNode::imageCallback, this, _1));

    publishVisualization_ =
        this->declare_parameter("publish_visualization", true);
    dedup_dist_thresh_ = this->declare_parameter("dedup_dist_thresh", 10.0);
    dedup_angle_thresh_ =
        this->declare_parameter("dedup_angle_thresh", 0.17);  // ~10 degrees
    if (publishVisualization_) {
        imageVisualizationPub_ =
            this->create_publisher<sensor_msgs::msg::Image>(
                image_visualization_pub_topic, qos_profile);
    }
    linesPub_ = this->create_publisher<vortex_msgs::msg::LineSegment2DArray>(
        lines_pub_topic, qos_profile);

    pipeline_ = LinedetectorPipe(fetchParams());
}

void PipelineLineFittingNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img;
    try {
        img = cv_bridge::toCvCopy(msg, "mono8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<Line> lines = pipeline_.detect(img, 2);

    // Remove near-duplicate lines (same pipe fitted twice by RANSAC).
    // Keeps the first (highest-scoring) line when midpoint distance and angle
    // difference are both below threshold.
    std::vector<Line> deduped;
    for (const auto& cand : lines) {
        double mx_c = (cand.start.x + cand.end.x) / 2.0;
        double my_c = (cand.start.y + cand.end.y) / 2.0;
        double ang_c =
            std::atan2(cand.end.y - cand.start.y, cand.end.x - cand.start.x);
        bool duplicate = false;
        for (const auto& kept : deduped) {
            double mx_k = (kept.start.x + kept.end.x) / 2.0;
            double my_k = (kept.start.y + kept.end.y) / 2.0;
            double ang_k = std::atan2(kept.end.y - kept.start.y,
                                      kept.end.x - kept.start.x);
            double dist = std::hypot(mx_c - mx_k, my_c - my_k);
            double dang = std::fmod(std::fabs(ang_c - ang_k), M_PI);
            if (dang > M_PI / 2.0)
                dang = M_PI - dang;
            if (dist < dedup_dist_thresh_ && dang < dedup_angle_thresh_) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
            deduped.push_back(cand);
    }
    lines = deduped;

    vortex_msgs::msg::LineSegment2DArray message;
    message.header = msg->header;

    for (const auto& line : lines) {
        vortex_msgs::msg::LineSegment2D seg;
        seg.p0.x = line.start.x;
        seg.p0.y = line.start.y;
        seg.p1.x = line.end.x;
        seg.p1.y = line.end.y;
        message.lines.push_back(seg);
    }

    linesPub_->publish(message);

    if (publishVisualization_) {
        auto img_color = drawLines(img, lines);
        auto output_image =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_color)
                .toImageMsg();
        output_image->header = msg->header;
        imageVisualizationPub_->publish(*output_image);
    }
}

cv::Mat PipelineLineFittingNode::drawLines(cv::Mat& image,
                                           const std::vector<Line>& lines) {
    cv::Mat img_color;
    // pipeline_.preprocess(image);
    cv::cvtColor(image, img_color, cv::COLOR_GRAY2BGR);

    cv::Mat img_with_lines = img_color.clone();

    for (const auto& line : lines) {
        cv::Point start(line.start.x, line.start.y);
        cv::Point end(line.end.x, line.end.y);
        cv::line(img_color, start, end, cv::Scalar(255, 0, 255), 10);
    }
    cv::addWeighted(img_with_lines, 0.5, img_color, 0.5, 0, img_color);

    return img_color;
}

RCLCPP_COMPONENTS_REGISTER_NODE(PipelineLineFittingNode)
