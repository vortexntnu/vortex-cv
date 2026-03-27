#ifndef BEARING_LOCALIZATION__LIB__GEOMETRY_CHECKS_HPP_
#define BEARING_LOCALIZATION__LIB__GEOMETRY_CHECKS_HPP_

#include "bearing_localization/lib/ray_measurement.hpp"

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

    static CheckResult check_minimum_rays(
        const std::vector<RayMeasurement>& rays,
        size_t min_rays);

    static CheckResult check_minimum_baseline(
        const std::vector<RayMeasurement>& rays,
        double min_baseline_m);

    static CheckResult check_angular_spread(
        const std::vector<RayMeasurement>& rays,
        double min_angle_deg);

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

#endif  // BEARING_LOCALIZATION__LIB__GEOMETRY_CHECKS_HPP_
