#pragma once

#include "bearing_localization/ray_measurement.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace bearing_localization {

class GeometryChecks {
   public:
    struct CheckResult {
        bool passed{false};
        std::string reason;
    };

    /// Require at least min_rays measurements.
    static CheckResult check_minimum_rays(
        const std::vector<RayMeasurement>& rays,
        size_t min_rays);

    /// Require the farthest pair of ray origins to be at least min_baseline_m
    /// apart.
    static CheckResult check_minimum_baseline(
        const std::vector<RayMeasurement>& rays,
        double min_baseline_m);

    /// Require the widest angle between any two ray directions to exceed
    /// min_angle_deg.
    static CheckResult check_angular_spread(
        const std::vector<RayMeasurement>& rays,
        double min_angle_deg);

    /// Require at least min_fraction of rays to have positive depth
    static CheckResult check_positive_depth(
        const std::vector<RayMeasurement>& rays,
        const Eigen::Vector3d& point,
        double min_fraction = 0.6);

    static CheckResult check_pre_solve(const std::vector<RayMeasurement>& rays,
                                       size_t min_rays,
                                       double min_baseline_m,
                                       double min_angle_deg);

    static CheckResult check_post_solve(const std::vector<RayMeasurement>& rays,
                                        const Eigen::Vector3d& point);
};

}  // namespace bearing_localization
