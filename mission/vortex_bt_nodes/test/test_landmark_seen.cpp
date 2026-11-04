#include <gtest/gtest.h>
#include <vortex_msgs/msg/landmark_track_array.hpp>

#include "bt_test_utils.hpp"
#include "vortex_bt_nodes/map/register.hpp"

namespace vortex_bt_nodes::test {
namespace {

using Tracks = vortex_msgs::msg::LandmarkTrackArray;

class LandmarkSeenTest : public BtNodeTest {
   protected:
    void SetUp() override {
        BtNodeTest::SetUp();
        map::register_nodes(factory_, client_node_);
        pub_ = server_node_->create_publisher<Tracks>(
            "landmark_server/landmarks",
            rclcpp::QoS(1).reliable().transient_local());
    }

    // Map time 100 s, one track per entry of how long ago it was seen.
    void publish(const std::vector<double>& seen_ago) {
        Tracks msg;
        msg.header.stamp.sec = 100;
        int id = 1;
        for (const double ago : seen_ago) {
            vortex_msgs::msg::LandmarkTrack t;
            t.landmark.id = id++;
            t.landmark.type.value = vortex_msgs::msg::LandmarkType::SLALOM_PIPE;
            t.landmark.subtype.value =
                vortex_msgs::msg::LandmarkSubtype::SLALOM_PIPE_RED;
            t.last_measurement.sec = static_cast<int32_t>(100.0 - ago);
            msg.landmark_tracks.push_back(t);
        }
        pub_->publish(msg);
        for (int i = 0; i < 30; ++i) {
            executor_.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    BT::NodeStatus seen(const std::string& extra = "") {
        auto tree = make_tree(
            R"(<LandmarkSeen type="SLALOM_PIPE" subtype="SLALOM_PIPE_RED" )" +
            extra + "/>");
        return run(tree);
    }

    rclcpp::Publisher<Tracks>::SharedPtr pub_;
};

TEST_F(LandmarkSeenTest, NoMapIsNotSeen) {
    EXPECT_EQ(seen(), BT::NodeStatus::FAILURE);
}

TEST_F(LandmarkSeenTest, JustDetectedIsSeen) {
    publish({1.0});
    EXPECT_EQ(seen(), BT::NodeStatus::SUCCESS);
}

TEST_F(LandmarkSeenTest, KnownButOldIsNotSeen) {
    publish({30.0});
    EXPECT_EQ(seen(), BT::NodeStatus::FAILURE);
}

TEST_F(LandmarkSeenTest, TheNewestOfTheClassCounts) {
    publish({30.0, 1.0});
    EXPECT_EQ(seen(), BT::NodeStatus::SUCCESS);
}

TEST_F(LandmarkSeenTest, WithinSIsUsed) {
    publish({5.0});
    EXPECT_EQ(seen(R"(within_s="10")"), BT::NodeStatus::SUCCESS);
}

}  // namespace
}  // namespace vortex_bt_nodes::test
