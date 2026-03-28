#include "valve_detection/detection_utils.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace valve_detection {

BoundingBox undistort_bbox(const BoundingBox& bbox, const CameraIntrinsics& intr) {
    const cv::Mat K = (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.cx,
                                                  0, intr.fy, intr.cy,
                                                  0, 0, 1);
    const cv::Mat D = (cv::Mat_<double>(5, 1) << intr.dist_k1, intr.dist_k2,
                                                 intr.dist_p1, intr.dist_p2,
                                                 intr.dist_k3);

    const float cos_t = std::cos(bbox.theta);
    const float sin_t = std::sin(bbox.theta);
    const float hx = bbox.size_x * 0.5f;
    const float hy = bbox.size_y * 0.5f;
    const cv::Point2f c(bbox.center_x, bbox.center_y);

    // 4 edge midpoints of the OBB + center to anchor the undistorted box.
    std::vector<cv::Point2f> pts = {
        c,
        c + cv::Point2f( hx * cos_t,  hx * sin_t),
        c + cv::Point2f(-hx * cos_t, -hx * sin_t),
        c + cv::Point2f(-hy * sin_t,  hy * cos_t),
        c + cv::Point2f( hy * sin_t, -hy * cos_t),
    };

    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(pts, undistorted, K, D, cv::noArray(), K);

    cv::RotatedRect fitted = cv::minAreaRect(undistorted);

    BoundingBox result = bbox;
    result.center_x = fitted.center.x;
    result.center_y = fitted.center.y;
    result.size_x = fitted.size.width;
    result.size_y = fitted.size.height;
    result.theta = fitted.angle * static_cast<float>(M_PI) / 180.0f;
    return result;
}

std::vector<size_t> filter_duplicate_detections(
    const std::vector<std::pair<float, BoundingBox>>& scored_boxes,
    float iou_duplicate_threshold) {
    const size_t n = scored_boxes.size();
    if (n == 0)
        return {};

    std::vector<std::pair<float, size_t>> order;
    order.reserve(n);
    for (size_t i = 0; i < n; ++i)
        order.emplace_back(scored_boxes[i].first, i);
    std::sort(order.begin(), order.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    std::vector<size_t> kept;
    std::vector<bool> suppressed(n, false);

    for (size_t si = 0; si < order.size() && kept.size() < 2; ++si) {
        const size_t i = order[si].second;
        if (suppressed[i])
            continue;

        kept.push_back(i);

        const BoundingBox& bi = scored_boxes[i].second;
        const float ai = bi.size_x * bi.size_y;
        const float bx1i = bi.center_x - bi.size_x * 0.5f;
        const float by1i = bi.center_y - bi.size_y * 0.5f;
        const float bx2i = bi.center_x + bi.size_x * 0.5f;
        const float by2i = bi.center_y + bi.size_y * 0.5f;

        for (size_t sj = si + 1; sj < order.size(); ++sj) {
            const size_t j = order[sj].second;
            if (suppressed[j])
                continue;

            const BoundingBox& bj = scored_boxes[j].second;
            const float aj = bj.size_x * bj.size_y;
            const float bx1j = bj.center_x - bj.size_x * 0.5f;
            const float by1j = bj.center_y - bj.size_y * 0.5f;
            const float bx2j = bj.center_x + bj.size_x * 0.5f;
            const float by2j = bj.center_y + bj.size_y * 0.5f;

            const float ix1 = std::max(bx1i, bx1j);
            const float iy1 = std::max(by1i, by1j);
            const float ix2 = std::min(bx2i, bx2j);
            const float iy2 = std::min(by2i, by2j);

            if (ix2 <= ix1 || iy2 <= iy1)
                continue;

            const float inter = (ix2 - ix1) * (iy2 - iy1);
            const float iou = inter / (ai + aj - inter);
            const float iom = inter / std::min(ai, aj);

            if (iou > iou_duplicate_threshold || iom > 0.7f)
                suppressed[j] = true;
        }
    }

    return kept;
}

}  // namespace valve_detection
