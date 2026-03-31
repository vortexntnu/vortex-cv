#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <vector>

#include <docking_position_estimator/docking_position_estimator.hpp>
#include <vortex/utils/types.hpp>

namespace vortex::docking_position_estimator::test {

using vortex::utils::types::LineSegment2D;
using vortex::utils::types::Point2D;

namespace {

constexpr float kEps = 1e-4f;

Point2D make_point(float x, float y) {
    return Point2D{.x = x, .y = y};
}

LineSegment2D make_line(float x0, float y0, float x1, float y1) {
    return LineSegment2D{
        .p0 = make_point(x0, y0),
        .p1 = make_point(x1, y1),
    };
}

DockingPositionEstimatorConfig make_config() {
    DockingPositionEstimatorConfig config{};

    config.min_wall_distance_m = 0.2f;
    config.max_wall_distance_m = 10.0f;

    // Merk: disse brukes ikke i classify_wall() akkurat nå,
    // men settes likevel for komplett config.
    config.right_wall_max_y_m = 0.0f;
    config.far_wall_min_x_m = 0.0f;

    config.parallel_heading_angle_threshold_rad =
        20.0f * static_cast<float>(M_PI) / 180.0f;
    config.perpendicular_heading_angle_threshold_rad =
        70.0f * static_cast<float>(M_PI) / 180.0f;

    config.min_corner_angle_rad =
        70.0f * static_cast<float>(M_PI) / 180.0f;
    config.max_corner_angle_rad =
        110.0f * static_cast<float>(M_PI) / 180.0f;

    config.right_wall_offset_m = 1.0f;
    config.far_wall_offset_m = 1.5f;

    return config;
}

}  // namespace

class DockingPositionEstimatorTest : public ::testing::Test {
   protected:
    DockingPositionEstimatorTest()
        : config(make_config()), estimator(config) {}

    DockingPositionEstimatorConfig config;
    DockingPositionEstimator estimator;
};

// -----------------------------------------------------------------------------
// NED expectations:
// x = North, y = East
//
// heading = 0 rad  -> facing North
//   forward = +x
//   right   = +y
//
// heading = pi/2   -> facing East
//   forward = +y
//   right   = +x
// -----------------------------------------------------------------------------


TEST_F(DockingPositionEstimatorTest,
       FacingNorth_RightWallToEast_AndFarWallAhead_ProducesCorner) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;  // Facing north in NED

    // Right wall should be to the east of the drone => y > 0
    // and approximately parallel to heading => line parallel to +x.
    const LineSegment2D right_wall = make_line(0.0f, 2.0f, 10.0f, 2.0f);

    // Far wall should be ahead of the drone => x > 0
    // and approximately perpendicular to heading.
    const LineSegment2D far_wall = make_line(5.0f, -5.0f, 5.0f, 5.0f);

    const auto corners =
        estimator.find_corner_estimates({right_wall, far_wall},
                                        drone_pos,
                                        drone_heading);

    ASSERT_EQ(corners.size(), 1u);
    EXPECT_NEAR(corners.front().corner_point.x(), 5.0f, kEps);
    EXPECT_NEAR(corners.front().corner_point.y(), 2.0f, kEps);
}

TEST_F(DockingPositionEstimatorTest,
       FacingNorth_WallToWest_IsNotClassifiedAsRightWall) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;  // Facing north in NED

    // This wall is on the left/west side => y < 0
    const LineSegment2D left_wall = make_line(0.0f, -2.0f, 10.0f, -2.0f);

    const LineSegment2D far_wall = make_line(5.0f, -5.0f, 5.0f, 5.0f);

    const auto corners =
        estimator.find_corner_estimates({left_wall, far_wall},
                                        drone_pos,
                                        drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST_F(DockingPositionEstimatorTest,
       FacingNorth_FarWallBehindDrone_IsRejected) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;  // Facing north in NED

    const LineSegment2D right_wall = make_line(0.0f, 2.0f, 10.0f, 2.0f);

    // Behind the drone => x < 0
    const LineSegment2D far_wall_behind =
        make_line(-5.0f, -5.0f, -5.0f, 5.0f);

    const auto corners =
        estimator.find_corner_estimates({right_wall, far_wall_behind},
                                        drone_pos,
                                        drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST_F(DockingPositionEstimatorTest,
       FacingNorth_RightWallOutsideDistanceThreshold_IsRejected) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;  // Facing north in NED

    // Too far to the east
    const LineSegment2D right_wall_too_far =
        make_line(0.0f, 20.0f, 10.0f, 20.0f);

    const LineSegment2D far_wall = make_line(5.0f, -5.0f, 5.0f, 5.0f);

    const auto corners =
        estimator.find_corner_estimates({right_wall_too_far, far_wall},
                                        drone_pos,
                                        drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST_F(DockingPositionEstimatorTest,
       FacingNorth_ApproximatelyParallelAndPerpendicularWalls_AreAccepted) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;  // Facing north in NED

    // Slightly tilted but still close to heading-parallel
    const LineSegment2D right_wall = make_line(0.0f, 2.0f, 10.0f, 2.4f);

    // Slightly tilted but still close to perpendicular
    const LineSegment2D far_wall = make_line(5.0f, -5.0f, 5.4f, 5.0f);

    const auto corners =
        estimator.find_corner_estimates({right_wall, far_wall},
                                        drone_pos,
                                        drone_heading);

    EXPECT_EQ(corners.size(), 1u);
}

TEST_F(DockingPositionEstimatorTest,
       RejectsParallelWalls_WhenNoCornerCanBeFormed) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;

    // Both are roughly parallel to heading
    const LineSegment2D line0 = make_line(0.0f, 2.0f, 10.0f, 2.0f);
    const LineSegment2D line1 = make_line(0.0f, 4.0f, 10.0f, 4.0f);

    const auto corners =
        estimator.find_corner_estimates({line0, line1},
                                        drone_pos,
                                        drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST_F(DockingPositionEstimatorTest,
       SelectBestCorner_ReturnsClosestCornerToDrone) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};

    CornerEstimate near_corner{
        .right_wall = make_line(0.0f, 2.0f, 10.0f, 2.0f),
        .far_wall = make_line(5.0f, -5.0f, 5.0f, 5.0f),
        .corner_point = Eigen::Vector2f{5.0f, 2.0f},
    };

    CornerEstimate far_corner{
        .right_wall = make_line(0.0f, 4.0f, 10.0f, 4.0f),
        .far_wall = make_line(8.0f, -5.0f, 8.0f, 5.0f),
        .corner_point = Eigen::Vector2f{8.0f, 4.0f},
    };

    const auto best =
        estimator.select_best_corner({far_corner, near_corner}, drone_pos);

    EXPECT_NEAR(best.corner_point.x(), 5.0f, kEps);
    EXPECT_NEAR(best.corner_point.y(), 2.0f, kEps);
}

TEST_F(DockingPositionEstimatorTest,
       EstimateDockingPosition_OffsetsFromCornerTowardPoolInterior_FacingNorth) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};

    // Corner at (5, 2)
    // Right wall: y = 2
    // Far wall:   x = 5
    //
    // For drone at (0, 0):
    // - normal from right wall toward drone = (0, -1)
    // - normal from far wall toward drone   = (-1, 0)
    //
    // Expected docking:
    // (5, 2) + 1.0*(0, -1) + 1.5*(-1, 0) = (3.5, 1.0)

    CornerEstimate corner{
        .right_wall = make_line(0.0f, 2.0f, 10.0f, 2.0f),
        .far_wall = make_line(5.0f, -5.0f, 5.0f, 5.0f),
        .corner_point = Eigen::Vector2f{5.0f, 2.0f},
    };

    const Eigen::Vector2f docking =
        estimator.estimate_docking_position(corner, drone_pos);

    EXPECT_NEAR(docking.x(), 3.5f, kEps);
    EXPECT_NEAR(docking.y(), 1.0f, kEps);
}

TEST_F(DockingPositionEstimatorTest,
       FullPipeline_FindsNearestValidCornerAndEstimatesDocking_FacingNorth) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;

    const LineSegment2D right_wall_near =
        make_line(0.0f, 2.0f, 10.0f, 2.0f);
    const LineSegment2D far_wall_near =
        make_line(5.0f, -5.0f, 5.0f, 5.0f);

    const LineSegment2D right_wall_far =
        make_line(0.0f, 4.0f, 10.0f, 4.0f);
    const LineSegment2D far_wall_far =
        make_line(9.0f, -5.0f, 9.0f, 5.0f);

    // Noise: behind drone, should not contribute
    const LineSegment2D noise =
        make_line(-5.0f, -3.0f, -2.0f, -3.0f);

    const std::vector<LineSegment2D> lines{
        right_wall_near, far_wall_near,
        right_wall_far, far_wall_far,
        noise
    };

    const auto corners =
        estimator.find_corner_estimates(lines, drone_pos, drone_heading);

    ASSERT_GE(corners.size(), 2u);

    const auto best = estimator.select_best_corner(corners, drone_pos);
    const auto docking = estimator.estimate_docking_position(best, drone_pos);

    EXPECT_NEAR(best.corner_point.x(), 5.0f, kEps);
    EXPECT_NEAR(best.corner_point.y(), 2.0f, kEps);

    EXPECT_NEAR(docking.x(), 3.5f, kEps);
    EXPECT_NEAR(docking.y(), 1.0f, kEps);
}

}  // namespace vortex::docking_position_estimator::test