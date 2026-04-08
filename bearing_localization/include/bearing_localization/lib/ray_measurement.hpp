#ifndef BEARING_LOCALIZATION__LIB__RAY_MEASUREMENT_HPP_
#define BEARING_LOCALIZATION__LIB__RAY_MEASUREMENT_HPP_

#include <Eigen/Dense>

namespace bearing_localization {

/**
 * @brief A single bearing ray measurement expressed in world coordinates.
 *
 * Represents a unit-direction ray originating from a known position,
 * used as input to the triangulation solver.
 */
struct RayMeasurement {
    double stamp_sec{0.0};  // Measurement timestamp (seconds).
    Eigen::Vector3d origin_world{
        Eigen::Vector3d::Zero()};  // Ray origin in the world frame.
    Eigen::Vector3d direction_world{
        Eigen::Vector3d::Zero()};  // Unit direction in the world frame.
    double weight{1.0};            // Optional weight for the solver.
};

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__LIB__RAY_MEASUREMENT_HPP_
