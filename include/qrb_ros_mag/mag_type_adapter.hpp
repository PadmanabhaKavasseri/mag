/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef QRB_ROS_MAG__MAG_TYPE_ADAPTER_HPP_
#define QRB_ROS_MAG__MAG_TYPE_ADAPTER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "sensor_client.h"

namespace qrb
{
namespace ros
{
class MagTypeAdapter
{
public:
  explicit MagTypeAdapter(const sensor_msgs::msg::MagneticField& sensor_msgs_mag);

  MagTypeAdapter(const sensors_event_t& mag_event);

  // MagTypeAdapter(){};//? what is this for?

  std_msgs::msg::Header header; //should be header_
  sensors_event_t* sensor_ptr; //should be sensor_ptr_
};
} // namespace ros
} // namespace qrb

template <>
struct rclcpp::TypeAdapter<qrb::ros::MagTypeAdapter, sensor_msgs::msg::MagneticField>
{
  using is_specialized = std::true_type;
  using custom_type = qrb::ros::MagTypeAdapter;
  using ros_message_type = sensor_msgs::msg::MagneticField;

  static void convert_to_ros_message(const custom_type& source, ros_message_type& destination)
  {
    destination.header = source.header;

    destination.magnetic_field.x = source.sensor_ptr->magnetic.x;
    destination.magnetic_field.y = source.sensor_ptr->magnetic.y;
    destination.magnetic_field.z = source.sensor_ptr->magnetic.z;

  }

  static void convert_to_custom(const ros_message_type& source, custom_type& destination)
  {
    destination = qrb::ros::MagTypeAdapter(source);
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(qrb::ros::MagTypeAdapter, sensor_msgs::msg::MagneticField);

#endif