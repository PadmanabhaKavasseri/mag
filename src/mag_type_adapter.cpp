/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include "qrb_ros_mag/mag_type_adapter.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "sensor_client.h"

namespace qrb
{
namespace ros
{
MagTypeAdapter::MagTypeAdapter(const sensors_event_t& mag_event)
{
  RCLCPP_INFO(rclcpp::get_logger("QTI_MAG"), "Here9\n");
  if (mag_event.timestamp == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("QTI MAG"), "data error");
  }
  
  header.stamp.nanosec = mag_event.timestamp % 1000000000LL;
  header.stamp.sec = mag_event.timestamp / 1000000000LL;
  
  header.frame_id = "magnetic_field";
  // sensors_event_t* ptr = sensor_ptr;
  // *ptr = mag_event;
  sensor_ptr = mag_event;
  RCLCPP_INFO(rclcpp::get_logger("QTI_MAG"), "Here10\n");
}

MagTypeAdapter::MagTypeAdapter(const sensor_msgs::msg::MagneticField& sensor_msgs_mag)
{
  RCLCPP_INFO(rclcpp::get_logger("QTI_MAG"), "Here11\n");
  sensors_event_t* mag = &sensor_ptr;
  
  header.stamp = sensor_msgs_mag.header.stamp;
  header.frame_id = sensor_msgs_mag.header.frame_id;
  mag->timestamp = sensor_msgs_mag.header.stamp.nanosec + sensor_msgs_mag.header.stamp.sec * 1000000000LL;
  
  //we should have magnetic field in our struct as well
  mag->magnetic.x = sensor_msgs_mag.magnetic_field.x;
  mag->magnetic.y = sensor_msgs_mag.magnetic_field.x;
  mag->magnetic.z = sensor_msgs_mag.magnetic_field.x;
  RCLCPP_INFO(rclcpp::get_logger("QTI_MAG"), "Here12\n");
}

}  // namespace ros
}  // namespace qrb
