//Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
//SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_mag/mag_component.hpp"
#include "gtest/gtest.h"

class NodeTestSuite : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(NodeTestSuite, RosMessageTest1) {
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  uint16_t h = 0;
  auto pub = test_node->create_publisher<TOPIC_TYPE>(TOPIC_NAME, 10);
  auto sub = test_node->create_subscription<sensor_msgs::msg::MagneticField>(
      "/mag", 10, [&h](const sensor_msgs::msg::MagneticField::SharedPtr msg) { h = 1U; });

  EXPECT_EQ(pub->get_subscription_count(), 1U);
  EXPECT_EQ(sub->get_publisher_count(), 1U);

  auto message = sensor_msgs::msg::MagneticField();
  

  pub->publish(message);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  rclcpp::spin_some(test_node);

  EXPECT_EQ(h, 1U);

  pub.reset();
  sub.reset();
  test_node.reset();
}
