/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "qrb_ros_mag/mag_type_adapter.hpp"

namespace qrb
{
namespace ros
{
class MagListener : public rclcpp::Node
{
public:
  explicit MagListener(const rclcpp::NodeOptions& options) : Node("mag_test", options)
  {
    auto callback = [this](const qrb::ros::MagTypeAdapter& container) {
      long time_nanosec = (get_clock()->now()).nanoseconds();
      long receive_nanosec =
          container.header.stamp.sec * 1000000000LL + container.header.stamp.nanosec;
      RCLCPP_INFO(this->get_logger(), "receive mag data time: %ld", receive_nanosec);
      RCLCPP_INFO(this->get_logger(), "mag data x: %f, y: %f, z:%f",
                  container.sensor_ptr->magnetic.x, container.sensor_ptr->magnetic.y,
                  container.sensor_ptr->magnetic.z);
      RCLCPP_INFO(this->get_logger(), "receive mag data latency: %ld ns",
                  time_nanosec - receive_nanosec);
    };
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: mag");
    rclcpp::SubscriptionOptions sub_options;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    sub_ = create_subscription<qrb::ros::MagTypeAdapter>("mag", 40, callback, sub_options);
  }

private:
  rclcpp::Subscription<qrb::ros::MagTypeAdapter>::SharedPtr sub_;
};
}  // namespace ros
}  // namespace qrb

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto test_node = std::make_shared<qrb::ros::MagListener>(options);
  exec.add_node(test_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(qrb::ros::MagListener)