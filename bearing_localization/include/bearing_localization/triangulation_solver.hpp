#pragma once

#include "bearing_localization/ray_measurement.hpp"

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace bearing_localization {

struct SolverResult {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double mean_residual{0.0};
    std::vector<double> residuals;
    std::vector<size_t> inlier_indices;
    bool valid{false};
};

class TriangulationSolver {
   public:
    SolverResult solve_with_outlier_rejection(
        const std::vector<RayMeasurement>& rays,
        double residual_threshold_m,
        int max_iterations);

    std::vector<double> compute_residuals(
        const std::vector<RayMeasurement>& rays,
        const Eigen::Vector3d& point);

   private:
    std::optional<Eigen::Vector3d> solve_least_squares(
        const std::vector<RayMeasurement>& rays,
        const std::vector<size_t>& indices);
};

}  // namespace bearing_localization
