#include "bearing_localization/lib/bearing_localizer.hpp"

#include <spdlog/spdlog.h>

namespace bearing_localization {

BearingLocalizer::BearingLocalizer(const BearingLocalizationConfig& cfg)
    : cfg_(cfg),
      buffer_(static_cast<size_t>(cfg.window_size),
              cfg.max_measurement_age_sec) {}

void BearingLocalizer::add_measurement(const RayMeasurement& ray) {
    buffer_.add(ray);
}

std::optional<LocalizationResult> BearingLocalizer::solve(double now_sec) {
    buffer_.prune(now_sec);
    const auto& rays = buffer_.rays();

    if (static_cast<int>(rays.size()) < cfg_.min_measurements) {
        return std::nullopt;
    }

    const auto pre_check = GeometryChecks::check_pre_solve(
        rays, static_cast<size_t>(cfg_.min_measurements), cfg_.min_baseline_m,
        cfg_.min_ray_angle_deg);
    if (!pre_check.passed) {
        spdlog::debug("Pre-solve check failed: {}", pre_check.reason);
        return std::nullopt;
    }

    const SolverResult result = solver_.solve_with_outlier_rejection(
        rays, cfg_.outlier_residual_threshold_m, cfg_.max_outlier_iterations);

    if (!result.valid) {
        spdlog::warn("Solver returned invalid result.");
        return std::nullopt;
    }

    std::vector<RayMeasurement> inlier_rays;
    inlier_rays.reserve(result.inlier_indices.size());
    for (size_t idx : result.inlier_indices) {
        inlier_rays.push_back(rays[idx]);
    }

    const auto depth_check =
        GeometryChecks::check_post_solve(inlier_rays, result.position);
    if (!depth_check.passed) {
        spdlog::debug("Depth check failed: {}", depth_check.reason);
        return std::nullopt;
    }

    LocalizationResult loc_result;
    loc_result.position = result.position;
    loc_result.mean_residual = result.mean_residual;
    loc_result.inlier_rays = std::move(inlier_rays);
    return loc_result;
}

}  // namespace bearing_localization
