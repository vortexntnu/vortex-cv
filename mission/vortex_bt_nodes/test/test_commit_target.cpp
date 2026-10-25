#include <gtest/gtest.h>
#include <rcl/time.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "bt_test_utils.hpp"
#include "vortex_bt_nodes/map/register.hpp"

namespace vortex_bt_nodes::test {
namespace {

class CommitTargetTest : public BtNodeTest {
   protected:
    void SetUp() override {
        BtNodeTest::SetUp();
        executor_.remove_node(client_node_);
        client_node_ =
            rclcpp::Node::make_shared("commit_client", "/commit_test");
        executor_.add_node(client_node_);
        ASSERT_EQ(rcl_enable_ros_time_override(
                      client_node_->get_clock()->get_clock_handle()),
                  RCL_RET_OK);
        set_time(1.0);
        map::register_nodes(factory_, client_node_);
        broadcaster_ =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(server_node_);
    }

    void set_time(double seconds) {
        ASSERT_EQ(rcl_set_ros_time_override(
                      client_node_->get_clock()->get_clock_handle(),
                      static_cast<int64_t>(seconds * 1e9)),
                  RCL_RET_OK);
    }

    void publish_pose(double x = 2.0,
                      double y = 3.0,
                      double z = -1.0,
                      bool available = true) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = server_node_->now();
        tf.header.frame_id = available ? "commit_test/odom" : "disconnected";
        tf.child_frame_id = "commit_test/target";
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = z;
        tf.transform.rotation.z = std::sin(0.3);
        tf.transform.rotation.w = std::cos(0.3);
        broadcaster_->sendTransform(tf);
        for (int i = 0; i < 30; ++i) {
            executor_.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

TEST_F(CommitTargetTest, CommitsFullOdomPoseAfterStableRosTime) {
    publish_pose();
    auto tree = make_tree(
        R"(<CommitTarget frame="target" stable_m="0.2" stable_s="2" pose="{target_pose}"/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    Pose pose;
    EXPECT_FALSE(blackboard()->get("target_pose", pose));
    set_time(2.9);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(3.0);
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(blackboard()->get("target_pose", pose));
    EXPECT_DOUBLE_EQ(pose.position.x, 2.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 3.0);
    EXPECT_DOUBLE_EQ(pose.position.z, -1.0);
    EXPECT_NEAR(pose.orientation.z, std::sin(0.3), 1e-9);
    EXPECT_NEAR(pose.orientation.w, std::cos(0.3), 1e-9);
}

TEST_F(CommitTargetTest, MissingFrameWaitsAndStartsTimerWhenItAppears) {
    auto tree = make_tree(
        R"(<CommitTarget frame="target" stable_m="0.2" stable_s="2" pose="{target_pose}"/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(100.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    publish_pose();
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(101.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(102.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(CommitTargetTest, CumulativeDriftRestartsTheStabilityWindow) {
    publish_pose();
    auto tree = make_tree(
        R"(<CommitTarget frame="target" stable_m="0.2" stable_s="2" pose="{target_pose}"/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(2.0);
    publish_pose(2.125);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(3.0);
    publish_pose(2.25);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(4.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(5.0);
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(blackboard()->get<Pose>("target_pose").position.x, 2.25);
}

TEST_F(CommitTargetTest, UsesThreeDimensionalDistanceAndCommitsLatestPose) {
    publish_pose();
    auto tree = make_tree(
        R"(<CommitTarget frame="target" stable_m="0.25" stable_s="2" pose="{target_pose}"/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(2.0);
    publish_pose(2.0, 3.0, -0.5);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(3.0);
    publish_pose(2.0, 3.0, -0.25);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(4.0);
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(blackboard()->get<Pose>("target_pose").position.z, -0.25);
    publish_pose(10.0);
    EXPECT_DOUBLE_EQ(blackboard()->get<Pose>("target_pose").position.x, 2.0);
}

TEST_F(CommitTargetTest, MissingTransformRestartsTheStabilityWindow) {
    publish_pose();
    auto tree = make_tree(
        R"(<CommitTarget frame="target" stable_m="0.2" stable_s="2" pose="{target_pose}"/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(2.0);
    publish_pose(2.0, 3.0, -1.0, false);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(3.0);
    publish_pose();
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(4.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(5.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(CommitTargetTest, HaltAndRestartRereadsInputsAndResetsTimer) {
    publish_pose();
    blackboard()->set("duration", 2.0);
    auto tree = make_tree(
        R"(<CommitTarget frame="target" stable_m="0.2" stable_s="{duration}" pose="{target_pose}"/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(2.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    tree.haltTree();
    blackboard()->set("duration", 3.0);
    set_time(5.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(7.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(8.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(CommitTargetTest, ZeroThresholdsAndQualifiedFrameCommitImmediately) {
    publish_pose();
    auto tree = make_tree(
        R"(<CommitTarget frame="commit_test/target" stable_m="0" stable_s="0" pose="{target_pose}"/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(CommitTargetTest, RestartsStabilityAfterRosClockRewinds) {
    publish_pose();
    auto tree = make_tree(
        R"(<CommitTarget frame="target" stable_m="0.2" stable_s="2" pose="{target_pose}"/>)");
    set_time(10.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(11.5);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(11.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(12.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    set_time(13.0);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(CommitTargetTest, MissingOutputMappingFails) {
    publish_pose();
    auto tree = make_tree(
        R"(<CommitTarget frame="target" stable_m="0" stable_s="0"/>)");
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

class InvalidCommitTargetTest
    : public CommitTargetTest,
      public ::testing::WithParamInterface<const char*> {};

TEST_P(InvalidCommitTargetTest, FailsWithoutThrowing) {
    auto tree = make_tree(GetParam());
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    Pose pose;
    EXPECT_FALSE(blackboard()->get("target_pose", pose));
}

INSTANTIATE_TEST_SUITE_P(
    Inputs,
    InvalidCommitTargetTest,
    ::testing::Values(
        R"(<CommitTarget stable_m="0.2" stable_s="1" pose="{target_pose}"/>)",
        R"(<CommitTarget frame="" stable_m="0.2" stable_s="1" pose="{target_pose}"/>)",
        R"(<CommitTarget frame="target" stable_s="1" pose="{target_pose}"/>)",
        R"(<CommitTarget frame="target" stable_m="0.2" pose="{target_pose}"/>)",
        R"(<CommitTarget frame="target" stable_m="-1" stable_s="1" pose="{target_pose}"/>)",
        R"(<CommitTarget frame="target" stable_m="0.2" stable_s="-1" pose="{target_pose}"/>)",
        R"(<CommitTarget frame="target" stable_m="nan" stable_s="1" pose="{target_pose}"/>)",
        R"(<CommitTarget frame="target" stable_m="inf" stable_s="1" pose="{target_pose}"/>)",
        R"(<CommitTarget frame="target" stable_m="0.2" stable_s="nan" pose="{target_pose}"/>)",
        R"(<CommitTarget frame="target" stable_m="0.2" stable_s="inf" pose="{target_pose}"/>)",
        R"(<CommitTarget frame="{missing}" stable_m="0.2" stable_s="1" pose="{target_pose}"/>)"));

}  // namespace
}  // namespace vortex_bt_nodes::test
