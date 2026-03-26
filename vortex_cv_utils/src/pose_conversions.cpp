#include <vortex/cv_utils/pose_conversions.hpp>

namespace vortex::cv_utils {

Eigen::Quaterniond quaternion_from_rvec(const cv::Vec3d& rvec) {
    const Eigen::Vector3d axis_angle(rvec[0], rvec[1], rvec[2]);
    const double angle = axis_angle.norm();

    if (angle < Eigen::NumTraits<double>::epsilon()) {
        return Eigen::Quaterniond::Identity();
    }

    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis_angle / angle))
        .normalized();
}

vortex::utils::types::Pose pose_from_rvec_tvec(const cv::Vec3d& rvec,
                                               const cv::Vec3d& tvec) {
    const Eigen::Quaterniond q = quaternion_from_rvec(rvec);

    return {
        .x = tvec[0],
        .y = tvec[1],
        .z = tvec[2],
        .qw = q.w(),
        .qx = q.x(),
        .qy = q.y(),
        .qz = q.z(),
    };
}

}  // namespace vortex::cv_utils
