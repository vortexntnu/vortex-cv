#include <gtest/gtest.h>
#include <cmath>
#include <opencv2/core.hpp>
#include <vortex/cv_utils/pose_conversions.hpp>

namespace vortex::cv_utils {

constexpr double kTol = 1e-9;

// ---------------------------------------------------------------------------
// quaternion_from_rvec
// ---------------------------------------------------------------------------

TEST(QuaternionFromRvec, IdentityRotation) {
    auto q = quaternion_from_rvec({0.0, 0.0, 0.0});

    EXPECT_NEAR(q.w(), 1.0, kTol);
    EXPECT_NEAR(q.x(), 0.0, kTol);
    EXPECT_NEAR(q.y(), 0.0, kTol);
    EXPECT_NEAR(q.z(), 0.0, kTol);
}

TEST(QuaternionFromRvec, Rotation90DegAroundZ) {
    double angle = M_PI / 2.0;
    auto q = quaternion_from_rvec({0.0, 0.0, angle});

    EXPECT_NEAR(q.w(), std::cos(angle / 2.0), kTol);
    EXPECT_NEAR(q.x(), 0.0, kTol);
    EXPECT_NEAR(q.y(), 0.0, kTol);
    EXPECT_NEAR(q.z(), std::sin(angle / 2.0), kTol);
}

TEST(QuaternionFromRvec, Rotation90DegAroundX) {
    double angle = M_PI / 2.0;
    auto q = quaternion_from_rvec({angle, 0.0, 0.0});

    EXPECT_NEAR(q.w(), std::cos(angle / 2.0), kTol);
    EXPECT_NEAR(q.x(), std::sin(angle / 2.0), kTol);
    EXPECT_NEAR(q.y(), 0.0, kTol);
    EXPECT_NEAR(q.z(), 0.0, kTol);
}

TEST(QuaternionFromRvec, Rotation90DegAroundY) {
    double angle = M_PI / 2.0;
    auto q = quaternion_from_rvec({0.0, angle, 0.0});

    EXPECT_NEAR(q.w(), std::cos(angle / 2.0), kTol);
    EXPECT_NEAR(q.x(), 0.0, kTol);
    EXPECT_NEAR(q.y(), std::sin(angle / 2.0), kTol);
    EXPECT_NEAR(q.z(), 0.0, kTol);
}

TEST(QuaternionFromRvec, Rotation180DegAroundZ) {
    auto q = quaternion_from_rvec({0.0, 0.0, M_PI});

    EXPECT_NEAR(q.w(), 0.0, kTol);
    EXPECT_NEAR(q.x(), 0.0, kTol);
    EXPECT_NEAR(q.y(), 0.0, kTol);
    EXPECT_NEAR(std::abs(q.z()), 1.0, kTol);
}

TEST(QuaternionFromRvec, UnitNorm) {
    auto q = quaternion_from_rvec({0.3, -0.7, 1.2});
    EXPECT_NEAR(q.norm(), 1.0, kTol);
}

// ---------------------------------------------------------------------------
// pose_from_rvec_tvec
// ---------------------------------------------------------------------------

TEST(PoseFromRvecTvec, TranslationPassthrough) {
    auto pose = pose_from_rvec_tvec({0.0, 0.0, 0.0}, {1.5, -2.3, 4.7});

    EXPECT_NEAR(pose.x, 1.5, kTol);
    EXPECT_NEAR(pose.y, -2.3, kTol);
    EXPECT_NEAR(pose.z, 4.7, kTol);
}

TEST(PoseFromRvecTvec, IdentityOrientation) {
    auto pose = pose_from_rvec_tvec({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});

    EXPECT_NEAR(pose.qw, 1.0, kTol);
    EXPECT_NEAR(pose.qx, 0.0, kTol);
    EXPECT_NEAR(pose.qy, 0.0, kTol);
    EXPECT_NEAR(pose.qz, 0.0, kTol);
}

TEST(PoseFromRvecTvec, CombinedRotationAndTranslation) {
    double angle = M_PI / 2.0;
    auto pose = pose_from_rvec_tvec({angle, 0.0, 0.0}, {1.0, 2.0, 3.0});

    EXPECT_NEAR(pose.x, 1.0, kTol);
    EXPECT_NEAR(pose.y, 2.0, kTol);
    EXPECT_NEAR(pose.z, 3.0, kTol);
    EXPECT_NEAR(pose.qw, std::cos(angle / 2.0), kTol);
    EXPECT_NEAR(pose.qx, std::sin(angle / 2.0), kTol);
    EXPECT_NEAR(pose.qy, 0.0, kTol);
    EXPECT_NEAR(pose.qz, 0.0, kTol);
}

}  // namespace vortex::cv_utils
