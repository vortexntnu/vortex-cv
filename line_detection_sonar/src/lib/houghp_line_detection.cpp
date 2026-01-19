#include "line_detection_sonar/lib/houghp_line_detection.hpp"

namespace vortex::line_detection {

Result HoughPLineDetector::detect(const cv::Mat& input_image, Mode mode) const {
    cv::Mat edge_image;
    detect_edges(input_image, edge_image);

    std::vector<cv::Vec4i> lines;
    detect_line_segments(edge_image, lines);

    Result r;
    r.line_segments = to_line_segments(lines);

    switch (mode) {
        case Mode::standard: {
            r.output = NoOutput{};
            break;
        }
        case Mode::visualize: {
            VisualizeOutput v;
            make_overlay_color(input_image, edge_image, lines, v.overlay_color);
            r.output = std::move(v);
            break;
        }
        case Mode::debug: {
            DebugOutput d;
            d.canny =
                edge_image;  // shallow copy; use edge_image.clone() if needed
            make_overlay_canny(edge_image, lines, d.overlay_canny);
            make_overlay_color(input_image, edge_image, lines, d.overlay_color);
            r.output = std::move(d);
            break;
        }
    }
    return r;
}

}  // namespace vortex::line_detection
