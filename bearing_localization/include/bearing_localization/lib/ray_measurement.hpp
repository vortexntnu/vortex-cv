#ifndef BEARING_LOCALIZATION__LIB__RAY_MEASUREMENT_HPP_
#define BEARING_LOCALIZATION__LIB__RAY_MEASUREMENT_HPP_

#include <Eigen/Dense>

namespace bearing_localization {

struct RayMeasurement {
    double stamp_sec{0.0};
    Eigen::Vector3d origin_world{Eigen::Vector3d::Zero()};
    Eigen::Vector3d direction_world{Eigen::Vector3d::Zero()};
    double weight{1.0};
};

}  // namespace bearing_localization

#endif  // BEARING_LOCALIZATION__LIB__RAY_MEASUREMENT_HPP_
