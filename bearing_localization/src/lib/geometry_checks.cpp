#include "bearing_localization/lib/geometry_checks.hpp"

#include <cmath>

namespace bearing_localization {

GeometryChecks::CheckResult GeometryChecks::check_minimum_rays(
    const std::vector<RayMeasurement>& rays,
    size_t min_rays) {
    if (rays.size() < min_rays) {
        return {false, "Not enough rays: " + std::to_string(rays.size()) +
                           " < " + std::to_string(min_rays)};
    }
    return {true, ""};
}

GeometryChecks::CheckResult GeometryChecks::check_minimum_baseline(
    const std::vector<RayMeasurement>& rays,
    double min_baseline_m) {
    if (rays.size() < 2) {
        return {false, "Need at least 2 rays for baseline check"};
    }

    double max_dist = 0.0;
    for (size_t i = 0; i < rays.size(); ++i) {
        for (size_t j = i + 1; j < rays.size(); ++j) {
            double dist = (rays[i].origin_world - rays[j].origin_world).norm();
            if (dist > max_dist)
                max_dist = dist;
        }
    }

    if (max_dist < min_baseline_m) {
        return {false,
                "Insufficient baseline: " + std::to_string(max_dist) + " m"};
    }
    return {true, ""};
}

GeometryChecks::CheckResult GeometryChecks::check_angular_spread(
    const std::vector<RayMeasurement>& rays,
    double min_angle_deg) {
    if (rays.size() < 2) {
        return {false, "Need at least 2 rays for angular spread check"};
    }

    const double min_angle_rad = min_angle_deg * M_PI / 180.0;
    double max_angle = 0.0;

    for (size_t i = 0; i < rays.size(); ++i) {
        for (size_t j = i + 1; j < rays.size(); ++j) {
            double dot = rays[i].direction_world.dot(rays[j].direction_world);
            dot = std::max(-1.0, std::min(1.0, dot));
            double angle = std::acos(std::abs(dot));
            if (angle > max_angle)
                max_angle = angle;
        }
    }

    if (max_angle < min_angle_rad) {
        return {false, "Insufficient angular spread: " +
                           std::to_string(max_angle * 180.0 / M_PI) + " deg"};
    }
    return {true, ""};
}

GeometryChecks::CheckResult GeometryChecks::check_positive_depth(
    const std::vector<RayMeasurement>& rays,
    const Eigen::Vector3d& point,
    double min_fraction) {
    if (rays.empty()) {
        return {false, "No rays"};
    }

    size_t positive = 0;
    for (const auto& ray : rays) {
        if (ray.direction_world.dot(point - ray.origin_world) > 0.0) {
            ++positive;
        }
    }

    const double fraction =
        static_cast<double>(positive) / static_cast<double>(rays.size());
    if (fraction < min_fraction) {
        return {false, "Too many rays with negative depth: " +
                           std::to_string(fraction * 100.0) + "% positive"};
    }
    return {true, ""};
}

GeometryChecks::CheckResult GeometryChecks::check_pre_solve(
    const std::vector<RayMeasurement>& rays,
    size_t min_rays,
    double min_baseline_m,
    double min_angle_deg) {
    auto r = check_minimum_rays(rays, min_rays);
    if (!r.passed)
        return r;

    r = check_minimum_baseline(rays, min_baseline_m);
    if (!r.passed)
        return r;

    r = check_angular_spread(rays, min_angle_deg);
    return r;
}

GeometryChecks::CheckResult GeometryChecks::check_post_solve(
    const std::vector<RayMeasurement>& rays,
    const Eigen::Vector3d& point) {
    return check_positive_depth(rays, point);
}

}  // namespace bearing_localization
