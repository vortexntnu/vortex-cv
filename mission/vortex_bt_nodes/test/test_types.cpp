#include <gtest/gtest.h>

#include <vortex_msgs/msg/waypoint_mode.hpp>

#include "vortex_bt_nodes/common/types.hpp"

namespace vbn = vortex_bt_nodes;

TEST(TypesTest, PoseFromString) {
    const auto pose = BT::convertFromString<vbn::Pose>("1.0;-2.0;1.5;90");
    EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(pose.position.y, -2.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 1.5);
    EXPECT_NEAR(vbn::yaw_of(pose), M_PI / 2.0, 1e-9);
    EXPECT_THROW(BT::convertFromString<vbn::Pose>("1;2"), BT::RuntimeError);
}

TEST(TypesTest, PoseListFromString) {
    const auto poses =
        BT::convertFromString<vbn::PoseList>("1;0;1 | 2; 0; 1; 90");
    ASSERT_EQ(poses.size(), 2u);
    EXPECT_DOUBLE_EQ(poses[1].position.x, 2.0);
    EXPECT_NEAR(vbn::yaw_of(poses[1]), M_PI / 2.0, 1e-9);
}

TEST(TypesTest, LandmarkNames) {
    EXPECT_EQ(vbn::landmark_type_from_string("SLALOM_PIPE"),
              vortex_msgs::msg::LandmarkType::SLALOM_PIPE);
    EXPECT_EQ(vbn::landmark_subtype_from_string("SLALOM_PIPE_RED"),
              vortex_msgs::msg::LandmarkSubtype::SLALOM_PIPE_RED);
    EXPECT_EQ(vbn::landmark_subtype_from_string("ANY"), -1);
    EXPECT_FALSE(vbn::landmark_type_from_string("NOPE"));
    EXPECT_FALSE(vbn::landmark_subtype_from_string("NOPE"));
}

TEST(TypesTest, WaypointModeNames) {
    EXPECT_EQ(vbn::waypoint_mode_from_string("POSITION_AND_YAW"),
              vortex_msgs::msg::WaypointMode::POSITION_AND_YAW);
    EXPECT_EQ(vbn::waypoint_mode_from_string("ONLY_Z"),
              vortex_msgs::msg::WaypointMode::ONLY_Z);
    EXPECT_FALSE(vbn::waypoint_mode_from_string("NOPE"));
}

TEST(TypesTest, Matches) {
    vbn::LandmarkTrack track;
    track.landmark.type.value = vortex_msgs::msg::LandmarkType::GATE;
    track.landmark.subtype.value =
        vortex_msgs::msg::LandmarkSubtype::GATE_WHOLE;
    EXPECT_TRUE(vbn::matches(track, vortex_msgs::msg::LandmarkType::GATE, -1));
    EXPECT_TRUE(vbn::matches(track, vortex_msgs::msg::LandmarkType::GATE,
                             vortex_msgs::msg::LandmarkSubtype::GATE_WHOLE));
    EXPECT_FALSE(vbn::matches(track, vortex_msgs::msg::LandmarkType::BIN, -1));
}

TEST(TypesTest, MissionClock) {
    vbn::MissionClock clock{rclcpp::Time(100, 0), 900.0};
    EXPECT_DOUBLE_EQ(clock.elapsed_s(rclcpp::Time(160, 0)), 60.0);
    EXPECT_DOUBLE_EQ(clock.remaining_s(rclcpp::Time(160, 0)), 840.0);
}
