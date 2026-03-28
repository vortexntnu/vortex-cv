#ifndef VALVE_DETECTION__DETECTION_UTILS_HPP_
#define VALVE_DETECTION__DETECTION_UTILS_HPP_

#include <utility>
#include <vector>
#include "valve_detection/types.hpp"

namespace valve_detection {

/**
 * @brief Returns an undistorted copy of bbox.
 *
 * Undistorts the 4 edge midpoints of the OBB and the center point using the
 * given intrinsics, then refits an oriented bounding box to the result. Using
 * edge midpoints (rather than corners) produces a better fit around circular
 * objects such as valves.
 */
BoundingBox undistort_bbox(const BoundingBox& bbox,
                           const CameraIntrinsics& intr);

/**
 * @brief Greedy NMS: returns indices of kept detections (max 2).
 *
 * scored_boxes: (score, bbox) pairs. Two boxes are duplicates when IoMin
 * (intersection / min-area) or IoU exceeds iou_duplicate_threshold.
 */
std::vector<size_t> filter_duplicate_detections(
    const std::vector<std::pair<float, BoundingBox>>& scored_boxes,
    float iou_duplicate_threshold);

}  // namespace valve_detection

#endif  // VALVE_DETECTION__DETECTION_UTILS_HPP_
