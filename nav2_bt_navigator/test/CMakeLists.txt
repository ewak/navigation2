# bt action server test
ament_add_gtest(test_bt_action_server test_bt_action_server.cpp)
target_link_libraries(test_bt_action_server
  nav2_behavior_tree::nav2_dummy_behavior_action_bt_node
  nav2_behavior_tree::nav2_wait_action_bt_node
  bt_navigator_core
  rclcpp::rclcpp
)
