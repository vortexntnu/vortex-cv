#include <gtest/gtest.h>

#include "bt_test_utils.hpp"
#include "vortex_bt_nodes/map/set_map_focus.hpp"

using vortex_bt_nodes::map::SetMapFocus;
using vortex_bt_nodes::map::split_task_list;
using vortex_bt_nodes::test::BtNodeTest;

namespace {

class SetMapFocusTest : public BtNodeTest {
   protected:
    void SetUp() override {
        BtNodeTest::SetUp();
        factory_.registerNodeType<SetMapFocus>("SetMapFocus", client_node_);
    }

    // Stands in for landmark_server: refuses unknown task names.
    void start_focus_server() {
        srv_ = server_node_->create_service<SetMapFocus::Srv>(
            "landmark_server/set_focus",
            [this](const std::shared_ptr<SetMapFocus::Srv::Request> req,
                   std::shared_ptr<SetMapFocus::Srv::Response> res) {
                last_ = *req;
                ++calls_;
                res->success = true;
                for (const auto& t : req->tasks) {
                    if (t != "gate" && t != "torpedo") {
                        res->success = false;
                        res->message = "unknown task '" + t + "'";
                    }
                }
            });
    }

    rclcpp::Service<SetMapFocus::Srv>::SharedPtr srv_;
    SetMapFocus::Srv::Request last_;
    int calls_{0};
};

}  // namespace

TEST(SplitTaskList, TrimsAndDropsEmpty) {
    EXPECT_EQ(split_task_list(" gate, torpedo ,,"),
              (std::vector<std::string>{"gate", "torpedo"}));
    EXPECT_TRUE(split_task_list("").empty());
}

TEST_F(SetMapFocusTest, SendsTheFocus) {
    start_focus_server();
    auto tree = make_tree(R"(<SetMapFocus tasks="torpedo" commit="gate"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(calls_, 1);
    EXPECT_EQ(last_.tasks, std::vector<std::string>{"torpedo"});
    EXPECT_TRUE(last_.lock_others);
    EXPECT_EQ(last_.commit, std::vector<std::string>{"gate"});
}

TEST_F(SetMapFocusTest, FailsWhenRefused) {
    start_focus_server();
    auto tree = make_tree(R"(<SetMapFocus tasks="nope" lock_others="false"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(last_.lock_others);
}

TEST_F(SetMapFocusTest, FailsWithoutTheServer) {
    auto tree = make_tree(R"(<SetMapFocus tasks="gate" service_timeout_s="0.3"/>)");
    EXPECT_EQ(run(tree), BT::NodeStatus::FAILURE);
}
