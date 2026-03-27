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

struct LocalizationResult {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double mean_residual{0.0};
    std::vector<RayMeasurement> inlier_rays;
};

class BearingLocalizer {
   public:
    explicit BearingLocalizer(const BearingLocalizationConfig& cfg);

    void add_measurement(const RayMeasurement& ray);

    std::optional<LocalizationResult> solve(double now_sec);

   private:
    BearingLocalizationConfig cfg_;
    MeasurementBuffer buffer_;
    TriangulationSolver solver_;
};

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__LIB__BEARING_LOCALIZER_HPP_
