#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "bt_test_utils.hpp"
#include "vortex_bt_nodes/motion/register.hpp"

namespace vortex_bt_nodes::test {
namespace {

class SearchTest : public BtNodeTest {
   protected:
    void SetUp() override {
        BtNodeTest::SetUp();
        motion::register_nodes(factory_, client_node_);
        broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(server_node_);
    }

    void publish_pose() {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = server_node_->now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = 2.0;
        tf.transform.translation.z = -1.0;
        tf.transform.rotation.w = 1.0;
        broadcaster_->sendTransform(tf);
        for (int i = 0; i < 30; ++i) {
            executor_.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

TEST_F(SearchTest, FullRotationPausesAndFailsWhenExhausted) {
    start_server();
    publish_pose();
    auto tree = make_tree(R"(<Search pattern="ROTATE_STEPS" step_deg="90" pause_s="1.5"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
    ASSERT_EQ(server_->last_goal.waypoints.size(), 4u);
    for (std::size_t i = 0; i < 4; ++i) {
        const auto& wp = server_->last_goal.waypoints[i];
        EXPECT_DOUBLE_EQ(wp.pose.position.x, 2.0);
        EXPECT_DOUBLE_EQ(wp.pose.position.z, -1.0);
        EXPECT_DOUBLE_EQ(wp.hold_time_sec, 1.5);
        EXPECT_EQ(wp.waypoint_mode.mode, vortex_msgs::msg::WaypointMode::FULL_POSE);
        EXPECT_NEAR(wp.pose.orientation.z, std::sin((i + 1) * M_PI / 4), 1e-9);
        EXPECT_NEAR(wp.pose.orientation.w, std::cos((i + 1) * M_PI / 4), 1e-9);
    }
}

TEST_F(SearchTest, ArcStartsLeftAndEndsRightIncludingPartialStep) {
    start_server();
    publish_pose();
    auto tree = make_tree(R"(<Search pattern="SCAN_ARC" step_deg="40" arc_deg="100" pause_s="1"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
    const std::vector<double> headings{-40, -50, -10, 30, 50};
    ASSERT_EQ(server_->last_goal.waypoints.size(), headings.size());
    for (std::size_t i = 0; i < headings.size(); ++i) {
        const auto& q = server_->last_goal.waypoints[i].pose.orientation;
        EXPECT_NEAR(q.z, std::sin(headings[i] * M_PI / 360), 1e-9);
        EXPECT_NEAR(q.w, std::cos(headings[i] * M_PI / 360), 1e-9);
    }
}

TEST_F(SearchTest, MissingPoseFailsWithoutSendingGoal) {
    start_server();
    auto tree = make_tree(R"(<Search pattern="ROTATE_STEPS" step_deg="30" pause_s="0"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
    EXPECT_EQ(server_->goals_received, 0);
}

class InvalidSearchTest : public SearchTest,
                          public ::testing::WithParamInterface<const char*> {};

TEST_P(InvalidSearchTest, RejectsInvalidInputBeforeSending) {
    start_server();
    publish_pose();
    auto tree = make_tree(GetParam());
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
    EXPECT_EQ(server_->goals_received, 0);
}

INSTANTIATE_TEST_SUITE_P(
    Inputs, InvalidSearchTest,
    ::testing::Values(
        R"(<Search step_deg="30" pause_s="1"/>)",
        R"(<Search pattern="UNKNOWN" step_deg="30" pause_s="1"/>)",
        R"(<Search pattern="ROTATE_STEPS" pause_s="1"/>)",
        R"(<Search pattern="ROTATE_STEPS" step_deg="0" pause_s="1"/>)",
        R"(<Search pattern="ROTATE_STEPS" step_deg="180" pause_s="1"/>)",
        R"(<Search pattern="ROTATE_STEPS" step_deg="0.00001" pause_s="1"/>)",
        R"(<Search pattern="ROTATE_STEPS" step_deg="30"/>)",
        R"(<Search pattern="ROTATE_STEPS" step_deg="30" pause_s="-1"/>)",
        R"(<Search pattern="SCAN_ARC" step_deg="30" pause_s="1"/>)",
        R"(<Search pattern="SCAN_ARC" step_deg="30" arc_deg="0" pause_s="1"/>)",
        R"(<Search pattern="SCAN_ARC" step_deg="30" arc_deg="361" pause_s="1"/>)"));

TEST_F(SearchTest, HaltCancelsSweep) {
    start_server();
    publish_pose();
    server_->behaviour = FakeWaypointManager::Behaviour::kNeverFinish;
    auto tree = make_tree(R"(<Search pattern="ROTATE_STEPS" step_deg="30" pause_s="1"/>)");
    EXPECT_EQ(run(tree, 0.5), BT::NodeStatus::RUNNING);
    tree.haltTree();
    for (int i = 0; i < 30 && !server_->cancel_requested; ++i) {
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(server_->cancel_requested);
}

}  // namespace
}  // namespace vortex_bt_nodes::test
