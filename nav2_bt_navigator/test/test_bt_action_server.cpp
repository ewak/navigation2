// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2021 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/dummy_behavior.hpp"
#include "nav2_msgs/action/dummy_navigate.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "nav2_behavior_tree/utils/test_bt_action_server.hpp"
#include "nav2_behavior_tree/plugins/action/dummy_behavior_action.hpp"

class DummyBehaviorActionServer
  : public TestActionServer<nav2_msgs::action::DummyBehavior>
{
public:
  DummyBehaviorActionServer()
  : TestActionServer("dummy_behavior")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::DummyBehavior>> goal_handle)
  override
  {
    if (goal_handle) {
      const auto goal = goal_handle->get_goal();
      auto result = std::make_shared<nav2_msgs::action::DummyBehavior::Result>();

      if (goal->command.data == "trigger_unknown_error") {
        result->error_code = nav2_msgs::action::DummyBehavior::Result::UNKNOWN;
        result->error_msg = "unknown error";
        goal_handle->abort(result);
      } else if (goal->command.data == "trigger_tf_error") {
        result->error_code = nav2_msgs::action::DummyBehavior::Result::TF_ERROR;
        result->error_msg = "tf error";
        goal_handle->abort(result);
      } else {
        goal_handle->succeed(result);
      }
    }
  }
};

class DummyNavigateBtActionServer
  : public TestBtActionServer<nav2_msgs::action::DummyNavigate>
{
public:
  using ActionT = nav2_msgs::action::DummyNavigate;

  DummyNavigateBtActionServer(
    const std::string& node_name,
    const std::string& bt_action_name,
    const std::vector<std::string>& plugin_lib_names,
    const rclcpp::NodeOptions & options) :
      TestBtActionServer<nav2_msgs::action::DummyNavigate>(
        node_name,
        bt_action_name,
        plugin_lib_names,
        options)
  {}

  uint16_t inject_error_code = 0;
  std::string inject_error_msg = "";

protected:
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
      current_goal_ = goal;
      return true;
  }

  virtual void onLoop(void) override
  {
  }

  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr /*goal*/) override
  {
      current_goal_ = nullptr;
  }

  virtual void goalCompleted(
    nav2_msgs::action::DummyNavigate::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus /*final_bt_status*/) override
  {
    if (inject_error_code == 0) {
      result->error_code = inject_error_code;
      result->error_msg = inject_error_msg;
    }
  }

private:
  std::shared_ptr<const typename nav2_msgs::action::DummyNavigate::Goal> current_goal_;
};

class DummyNavigateTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
      // bt_action_server_node_->configure();
      // bt_action_server_node_->activate();
  }

  static void TearDownTestCase()
  {
      // bt_action_server_node_->deactivate();
      // bt_action_server_node_->cleanup();
  }

  void SetUp()
  {
    action_server_ = std::make_shared<DummyBehaviorActionServer>();
    rclcpp::NodeOptions  options;
    std::vector<std::string> plugin_lib_names = {
        "nav2_dummy_behavior_action_bt_node",
        "nav2_wait_action_bt_node"};
    bt_action_server_node_ =
      std::make_shared<DummyNavigateBtActionServer>(
        "dummy_navigator", "dummy_navigate", plugin_lib_names, options);
  }

  void TearDown() override
  {
    action_server_.reset();
    bt_action_server_node_.reset();
  }

  std::shared_ptr<DummyBehaviorActionServer> action_server_;
  std::shared_ptr<DummyNavigateBtActionServer> bt_action_server_node_;
};


TEST_F(DummyNavigateTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <DummyBehavior command="{command}" error_code_id="{dummy_behavior_error_code}" error_msg="{dummy_behavior_error_msg}""/>
        </BehaviorTree>
      </root>)";

  auto result = std::make_shared<nav2_msgs::action::DummyNavigate::Result>();
  bt_action_server_node_->configure();
  bt_action_server_node_->activate();

  bool has_internal_error = bt_action_server_node_->bt_action_server_->populateInternalError(result);
  EXPECT_EQ(has_internal_error, true);
  EXPECT_EQ(result->error_code, nav2_msgs::action::DummyNavigate::Result::FAILED_TO_LOAD_BEHAVIOR_TREE);
  EXPECT_EQ(result->error_msg, "Couldn't open input XML file: bt_xmL_not_found.xml");

  bt_action_server_node_->bt_action_server_->resetInternalError();
  has_internal_error = bt_action_server_node_->bt_action_server_->populateInternalError(result);
  EXPECT_EQ(has_internal_error, false);

  bt_action_server_node_->bt_action_server_->setInternalError(
    nav2_msgs::action::DummyNavigate::Result::UNKNOWN, "some test coverage");
  has_internal_error = bt_action_server_node_->bt_action_server_->populateInternalError(result);
  EXPECT_EQ(has_internal_error, true);
  EXPECT_EQ(result->error_code, nav2_msgs::action::DummyNavigate::Result::UNKNOWN);
  EXPECT_EQ(result->error_msg, "some test coverage");

  bt_action_server_node_->cleanup();
  bt_action_server_node_->shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
