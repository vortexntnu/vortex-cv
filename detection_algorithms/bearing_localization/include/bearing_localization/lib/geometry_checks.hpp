#ifndef BEARING_LOCALIZATION__LIB__GEOMETRY_CHECKS_HPP_
#define BEARING_LOCALIZATION__LIB__GEOMETRY_CHECKS_HPP_

#include "bearing_localization/lib/ray_measurement.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace bearing_localization {

/**
 * @brief Static utility class for geometric validation of ray sets.
 *
 * Provides pre-solve and post-solve sanity checks (minimum ray count,
 * baseline distance, angular spread, positive depth) to guard the
 * triangulation solver against degenerate configurations.
 */
class GeometryChecks {
   public:
    /**
     * @brief Result of a single geometry check.
     */
    struct CheckResult {
        bool passed{false};  // True if the check succeeded.
        std::string reason;  // failure reason.
    };

    /**
     * @brief Verify that enough rays are available.
     * @param rays   Ray measurements to check.
     * @param min_rays  Minimum number required.
     */
    static CheckResult check_minimum_rays(
        const std::vector<RayMeasurement>& rays,
        size_t min_rays);

    /**
     * @brief Verify that the spatial baseline between the first and last
     *        ray origins meets the minimum distance.
     * @param rays  Ray measurements to check.
     * @param min_baseline_m  Minimum baseline distance (m).
     */
    static CheckResult check_minimum_baseline(
        const std::vector<RayMeasurement>& rays,
        double min_baseline_m);

    /**
     * @brief Verify that the maximum angle between any two rays exceeds
     *        the minimum threshold.
     * @param rays  Ray measurements to check.
     * @param min_angle_deg  Minimum angular spread (degrees).
     */
    static CheckResult check_angular_spread(
        const std::vector<RayMeasurement>& rays,
        double min_angle_deg);

    /**
     * @brief Verify that a sufficient fraction of rays have the estimated
     *        target in front of them (positive depth).
     * @param rays  Ray measurements to check.
     * @param point  Estimated target position.
     * @param min_fraction  Fraction of rays that must pass (default 0.6).
     */
    static CheckResult check_positive_depth(
        const std::vector<RayMeasurement>& rays,
        const Eigen::Vector3d& point,
        double min_fraction = 0.6);

    /**
     * @brief Run all pre-solve checks (ray count, baseline, angular spread).
     * @return Combined CheckResult; fails on the first violated condition.
     */
    static CheckResult check_pre_solve(const std::vector<RayMeasurement>& rays,
                                       size_t min_rays,
                                       double min_baseline_m,
                                       double min_angle_deg);

    /**
     * @brief Run all post-solve checks (positive depth).
     * @param rays  Inlier rays used in the solution.
     * @param point  Estimated target position.
     */
    static CheckResult check_post_solve(const std::vector<RayMeasurement>& rays,
                                        const Eigen::Vector3d& point);
};

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__LIB__GEOMETRY_CHECKS_HPP_
