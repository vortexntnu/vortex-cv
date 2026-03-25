#include "bearing_localization/triangulation_solver.hpp"

#include <algorithm>
#include <numeric>

namespace bearing_localization {

SolverResult TriangulationSolver::solve_with_outlier_rejection(
    const std::vector<RayMeasurement>& rays,
    double residual_threshold_m,
    int max_iterations) {
    std::vector<size_t> inliers(rays.size());
    std::iota(inliers.begin(), inliers.end(), 0);

    SolverResult result;

    for (int iter = 0; iter <= max_iterations; ++iter) {
        if (inliers.empty()) {
            return result;
        }

        auto pos = solve_least_squares(rays, inliers);
        if (!pos) {
            return result;
        }

        std::vector<double> all_residuals = compute_residuals(rays, *pos);

        std::vector<size_t> new_inliers;
        new_inliers.reserve(inliers.size());
        for (size_t idx : inliers) {
            if (all_residuals[idx] <= residual_threshold_m) {
                new_inliers.push_back(idx);
            }
        }

        if (new_inliers.size() == inliers.size() || iter == max_iterations) {
            result.position = *pos;
            result.residuals = all_residuals;

            result.inlier_indices = new_inliers.empty() ? inliers : new_inliers;

            double sum = 0.0;
            for (size_t idx : result.inlier_indices)
                sum += all_residuals[idx];
            result.mean_residual =
                result.inlier_indices.empty()
                    ? 0.0
                    : sum / static_cast<double>(result.inlier_indices.size());
            result.valid = true;
            return result;
        }

        inliers = new_inliers;
    }

    return result;
}

std::vector<double> TriangulationSolver::compute_residuals(
    const std::vector<RayMeasurement>& rays,
    const Eigen::Vector3d& point) {
    std::vector<double> residuals;
    residuals.reserve(rays.size());
    for (const auto& ray : rays) {
        const Eigen::Matrix3d A =
            Eigen::Matrix3d::Identity() -
            ray.direction_world * ray.direction_world.transpose();
        residuals.push_back((A * (point - ray.origin_world)).norm());
    }
    return residuals;
}

std::optional<Eigen::Vector3d> TriangulationSolver::solve_least_squares(
    const std::vector<RayMeasurement>& rays,
    const std::vector<size_t>& indices) {
    if (indices.empty()) {
        return std::nullopt;
    }

    Eigen::Matrix3d A_sum = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b_sum = Eigen::Vector3d::Zero();

    for (size_t idx : indices) {
        const auto& ray = rays[idx];
        const Eigen::Matrix3d A =
            Eigen::Matrix3d::Identity() -
            ray.direction_world * ray.direction_world.transpose();
        A_sum += ray.weight * A;
        b_sum += ray.weight * A * ray.origin_world;
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        A_sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.rank() < 3) {
        return std::nullopt;
    }

    return svd.solve(b_sum);
}

}  // namespace bearing_localization
