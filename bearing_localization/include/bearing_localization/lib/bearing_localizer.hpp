#ifndef BEARING_LOCALIZATION__LIB__BEARING_LOCALIZER_HPP_
#define BEARING_LOCALIZATION__LIB__BEARING_LOCALIZER_HPP_

#include "bearing_localization/lib/bearing_localization_config.hpp"
#include "bearing_localization/lib/geometry_checks.hpp"
#include "bearing_localization/lib/measurement_buffer.hpp"
#include "bearing_localization/lib/ray_measurement.hpp"
#include "bearing_localization/lib/triangulation_solver.hpp"

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace bearing_localization {

/**
 * @brief Output of a successful bearing-only localisation solve.
 */
struct LocalizationResult {
    Eigen::Vector3d position{
        Eigen::Vector3d::Zero()};  // Estimated 3-D target position.
    double mean_residual{0.0};     // Mean point-to-ray residual (m).
    std::vector<RayMeasurement>
        inlier_rays;  // Rays retained after outlier rejection.
};

/**
 * @brief High-level bearing-only localiser.
 *
 * Buffers incoming ray measurements, runs geometry checks, and invokes the
 * triangulation solver with iterative outlier rejection to estimate the
 * 3-D position of a target observed from multiple viewpoints.
 */
class BearingLocalizer {
   public:
    explicit BearingLocalizer(const BearingLocalizationConfig& cfg);

    /**
     * @brief Add a new ray measurement to the internal buffer.
     * @param ray World-frame ray measurement.
     */
    void add_measurement(const RayMeasurement& ray);

    /**
     * @brief Attempt to triangulate the target position.
     *
     * Prunes stale measurements, runs geometry checks, solves via
     * least-squares with outlier rejection, and returns the result.
     *
     * @param now_sec Current time in seconds (used for pruning).
     * @return LocalizationResult on success, std::nullopt if checks fail.
     */
    std::optional<LocalizationResult> solve(double now_sec);

   private:
    BearingLocalizationConfig cfg_;
    MeasurementBuffer buffer_;
    TriangulationSolver solver_;
    std::optional<Eigen::Vector3d> prev_position_;
};

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__LIB__BEARING_LOCALIZER_HPP_
