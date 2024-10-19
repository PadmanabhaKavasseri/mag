/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef QRB_ROS_MAG__MAG_COMPONENT_HPP_
#define QRB_ROS_MAG__MAG_COMPONENT_HPP_

#include <qrb_ros_mag/mag_type_adapter.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_client.h"

#define TOPIC_NAME "mag"
#define TOPIC_TYPE qrb::ros::MagTypeAdapter
#define RETRY_MAX 30
#define RETRY_INTERVAL 2  // 2s

namespace qrb
{
namespace ros
{
class MagComponent: public rclcpp::Node
{
public:
  bool is_working;
  explicit MagComponent(const rclcpp::NodeOptions& options);
  ~MagComponent();
  
private:
  bool init();
  void connect_success();
  void retry_connection();
  void publish_msg();
  SensorClient sensor_client_;
  rclcpp::Publisher<TOPIC_TYPE>::SharedPtr publisher_;
  bool debug_ = false;
  bool running_;
  int retry_;
  std::shared_ptr<std::thread> thread_publish_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros
}  // namespace qrb

#endif