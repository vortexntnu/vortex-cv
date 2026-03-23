#ifndef VORTEX__CV_UTILS__POSE_CONVERSIONS_HPP_
#define VORTEX__CV_UTILS__POSE_CONVERSIONS_HPP_

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <vortex/utils/types.hpp>

namespace vortex::cv_utils {

/**
 * @brief Convert an OpenCV rotation vector to a quaternion.
 *
 * @param rvec Rotation vector (3x1).
 * @return Eigen::Quaterniond representing the rotation.
 */
Eigen::Quaterniond quaternion_from_rvec(const cv::Vec3d& rvec);

/**
 * @brief Convert an OpenCV rotation vector and translation vector to a Pose.
 *
 * @param rvec Rotation vector (3x1).
 * @param tvec Translation vector (3x1).
 * @return vortex::utils::types::Pose with position and orientation.
 */
vortex::utils::types::Pose pose_from_rvec_tvec(const cv::Vec3d& rvec,
                                               const cv::Vec3d& tvec);

}  // namespace vortex::cv_utils

#endif  // VORTEX__CV_UTILS__POSE_CONVERSIONS_HPP_
