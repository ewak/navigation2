// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__UTILS__TEST_BT_ACTION_SERVER_HPP_
#define NAV2_BEHAVIOR_TREE__UTILS__TEST_BT_ACTION_SERVER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"

template<class ActionT>
class TestBtActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit TestBtActionServer(
    const std::string& node_name,
    const std::string& bt_action_name,
    const std::vector<std::string>& plugin_lib_names,
    const rclcpp::NodeOptions & options) :
      rclcpp_lifecycle::LifecycleNode(node_name, options),
      bt_action_name_(bt_action_name),
      plugin_lib_names_(plugin_lib_names),
      default_bt_xml_filename_("bt_xmL_not_found.xml")
  {
  }

  /**
   * @brief Configuring of the navigator's backend BT and actions
   * @return bool If successful
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state)
  {
    using namespace std::placeholders;  // NOLINT
    std::string dummy_bt_xml = R"xml(
      <?xml version="1.0" encoding="UTF-8"?>
      <root BTCPP_format="4">
        <Sleep msec="300"/>
      </root>";
    )xml";
    this->bt_action_server_ = std::make_shared<nav2_behavior_tree::BtActionServer<ActionT>>(
      shared_from_this(),
      bt_action_name_,
      plugin_lib_names_,
      default_bt_xml_filename_,
      std::bind(&TestBtActionServer::onGoalReceived, this, std::placeholders::_1),
      std::bind(&TestBtActionServer::onLoop, this),
      std::bind(&TestBtActionServer::onPreempt, this, std::placeholders::_1),
      std::bind(&TestBtActionServer::onCompletion, this, std::placeholders::_1, std::placeholders::_2)
    );

    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      on_cleanup(state);
      ok = false;
    }

    return ok ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
  }

  /**
   * @brief Activation of the navigator's backend BT and actions
   * @return bool If successful
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    bool ok = true;
    if (!bt_action_server_->on_activate()) {
      ok = false;
    }

    return ok ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
  }

  /**
   * @brief Deactivation of the navigator's backend BT and actions
   * @return bool If successful
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    bool ok = true;
    if (!bt_action_server_->on_deactivate()) {
      ok = false;
    }

    return ok ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
  }

  /**
   * @brief Cleanup a navigator
   * @return bool If successful
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    bool ok = true;
    if (!bt_action_server_->on_cleanup()) {
      ok = false;
    }

    bt_action_server_.reset();

    return ok ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
  }

  std::shared_ptr<const typename ActionT::Goal> getCurrentGoal(void) const
  {
    return bt_action_server_->getCurrentGoal();
  }

  void setReturnSuccess(bool return_success)
  {
    return_success_ = return_success;
  }

  bool getReturnSuccess(void)
  {
    return return_success_;
  }

  bool isGoalCancelled()
  {
    return goal_cancelled_;
  }

  std::shared_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;

protected:
  /**
   * @brief An intermediate goal reception function
   */
  virtual bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
    bool goal_accepted = goalReceived(goal);
    return goal_accepted;
  }

  /**
   * @brief An intermediate completion function
   */
  virtual void onCompletion(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status)
  {
    goalCompleted(result, final_bt_status);
  }

  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   */
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  virtual void onLoop() = 0;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that is called when a the action is completed; Can fill in
   * action result message or indicate that this action is done.
   */
  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) = 0;

private:
  std::string bt_action_name_;
  std::vector<std::string> plugin_lib_names_;
  std::string default_bt_xml_filename_;
  bool return_success_ = true;
  bool goal_cancelled_ = false;
};

#endif  // NAV2_BEHAVIOR_TREE__UTILS__TEST_ACTION_SERVER_HPP_
