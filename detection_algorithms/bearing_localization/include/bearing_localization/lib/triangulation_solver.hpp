#ifndef BEARING_LOCALIZATION__LIB__TRIANGULATION_SOLVER_HPP_
#define BEARING_LOCALIZATION__LIB__TRIANGULATION_SOLVER_HPP_

#include "bearing_localization/lib/ray_measurement.hpp"

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace bearing_localization {

/**
 * @brief Output of the triangulation solver.
 */
struct SolverResult {
    Eigen::Vector3d position{
        Eigen::Vector3d::Zero()};        // Estimated 3-D position.
    double mean_residual{0.0};           // Mean point-to-ray distance (m).
    std::vector<double> residuals;       // Per-ray residuals.
    std::vector<size_t> inlier_indices;  // Indices of rays kept as inliers.
    bool valid{false};                   // True if the solution is usable.
};

/**
 * @brief Weighted least-squares triangulation solver with iterative outlier
 * rejection.
 *
 * Solves for the 3-D point that minimises the sum of squared distances to a
 * set of bearing rays.  Outliers are iteratively removed based on a residual
 * threshold until convergence or the iteration limit is reached.
 */
class TriangulationSolver {
   public:
    /**
     * @brief Solve for the target position with outlier rejection.
     * @param rays  Input ray measurements.
     * @param residual_threshold_m  Rays with residual above this are outliers
     * (m).
     * @param max_iterations  Maximum outlier-rejection iterations.
     * @return SolverResult with position, residuals, and inlier indices.
     */
    SolverResult solve_with_outlier_rejection(
        const std::vector<RayMeasurement>& rays,
        double residual_threshold_m,
        int max_iterations);

    /**
     * @brief Compute per-ray point-to-ray residuals for a given position.
     * @param rays  Ray measurements.
     * @param point  3-D point to evaluate.
     * @return Vector of residual distances (m), one per ray.
     */
    std::vector<double> compute_residuals(
        const std::vector<RayMeasurement>& rays,
        const Eigen::Vector3d& point);

   private:
    /**
     * @brief Solve the least-squares sub-problem over the given ray subset.
     * @param rays  Full ray vector.
     * @param indices  Indices into @p rays to include.
     * @return Estimated position, or std::nullopt if the system is singular.
     */
    std::optional<Eigen::Vector3d> solve_least_squares(
        const std::vector<RayMeasurement>& rays,
        const std::vector<size_t>& indices);
};

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__LIB__TRIANGULATION_SOLVER_HPP_
