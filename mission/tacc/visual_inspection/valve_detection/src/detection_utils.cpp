#include "valve_detection/detection_utils.hpp"

#include <algorithm>
#include <cmath>

namespace valve_detection {

// Greedy NMS (non-maximum suppression) that keeps at most 2 detections.
//
// Boxes are processed in descending confidence order. For each kept box,
// all remaining boxes that overlap it above the IoU threshold OR whose
// intersection-over-minimum (IoM) exceeds 70 % are suppressed. IoM
// catches the case where a small spurious detection sits inside a
// larger, higher-confidence one.
//
// Uses axis-aligned bounding rectangles for the overlap test (ignoring
// OBB rotation) since the YOLO OBB angles are small and the cost of
// exact OBB intersection is not justified here.
std::vector<size_t> filter_duplicate_detections(
    const std::vector<std::pair<float, BoundingBox>>& scored_boxes,
    float iou_duplicate_threshold) {
    const size_t n = scored_boxes.size();
    if (n == 0)
        return {};

    // Sort indices by descending confidence score.
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

        // Compute AABB of the kept box for overlap tests.
        const BoundingBox& bi = scored_boxes[i].second;
        const float ai = bi.size_x * bi.size_y;
        const float bx1i = bi.center_x - bi.size_x * 0.5f;
        const float by1i = bi.center_y - bi.size_y * 0.5f;
        const float bx2i = bi.center_x + bi.size_x * 0.5f;
        const float by2i = bi.center_y + bi.size_y * 0.5f;

        // Suppress lower-confidence boxes that overlap too much.
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

            // AABB intersection rectangle.
            const float ix1 = std::max(bx1i, bx1j);
            const float iy1 = std::max(by1i, by1j);
            const float ix2 = std::min(bx2i, bx2j);
            const float iy2 = std::min(by2i, by2j);

            if (ix2 <= ix1 || iy2 <= iy1)
                continue;  // no overlap

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
