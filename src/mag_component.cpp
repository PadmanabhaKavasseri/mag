/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "qrb_ros_mag/mag_component.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <thread>


#include "qrb_ros_mag/mag_type_adapter.hpp"
#include "sensor_client.h"

namespace qrb
{
namespace ros
{
MagComponent::MagComponent(const rclcpp::NodeOptions& options) : Node("mag_node", options)
{
  retry_ = 0;
  running_ = false;
  is_working = true;
  this->declare_parameter<bool>("debug", false);
  if (!this->init()) {
    RCLCPP_ERROR(this->get_logger(), "mag component init failed");
    RCLCPP_ERROR(this->get_logger(), "will try to reinit again");
  } else {
    RCLCPP_INFO(this->get_logger(), "mag component running...");
  }
  RCLCPP_INFO(this->get_logger(), "Here1\n");
}

MagComponent::~MagComponent()
{
  RCLCPP_INFO(this->get_logger(), "stopping...");
  running_ = false;
  is_working = false;
  if (thread_publish_msg_) {
    thread_publish_msg_->join();
  }
  sensor_client_.DisconnectServer();
}

bool MagComponent::init()
{
  if (!sensor_client_.CreateConnection()) {
    RCLCPP_ERROR(this->get_logger(), "mag client connect failed.");
    timer_ = this->create_wall_timer(std::chrono::seconds(RETRY_INTERVAL),
                                     std::bind(&MagComponent::retry_connection, this));
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Here2\n");
  connect_success();
  return true;
}

void MagComponent::connect_success()
{
  RCLCPP_INFO(this->get_logger(), "mag client connect success");
  running_ = true;

  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  publisher_ = this->create_publisher<TOPIC_TYPE>(TOPIC_NAME, 30, pub_options);
  thread_publish_msg_ =
      std::make_shared<std::thread>(std::mem_fn(&MagComponent::publish_msg), this);
  RCLCPP_INFO(this->get_logger(), "Here3\n");
}

void MagComponent::retry_connection()
{
  RCLCPP_INFO(this->get_logger(), "retry to connect");
  bool success = sensor_client_.CreateConnection();
  if (success && !running_) {
    connect_success();
    this->timer_->cancel();
    return;
  } else {
    RCLCPP_ERROR(this->get_logger(), "reconnect service failed");
    RCLCPP_ERROR(this->get_logger(), "retry %d/%d after %d seconds...", ++retry_, RETRY_MAX,
                 RETRY_INTERVAL);
  }
  if (retry_ >= RETRY_MAX) {
    RCLCPP_ERROR(this->get_logger(), "reconnect service failed");
    this->timer_->cancel();
    is_working = false;
  }
}

void MagComponent::publish_msg()
{
  int32_t pack_num = 0;
  int i = 0;
  sensors_event_t* mag_ptr;
  std::unique_ptr<qrb::ros::MagTypeAdapter> container;
  long time_nanosec;
  long long latency_sum = 0;
  RCLCPP_INFO(this->get_logger(), "Here4\n");
  while (running_) {
    RCLCPP_INFO(this->get_logger(), "Here5\n");
    pack_num = 0;
    this->get_parameter("debug", debug_);
    if (!sensor_client_.GetMagData(&mag_ptr, &pack_num)) {
      RCLCPP_DEBUG(this->get_logger(), "get mag data failed");
      if (!debug_) {
        RCLCPP_DEBUG(this->get_logger(), "thread will sleep 2 ms");
        std::this_thread::sleep_for(std::chrono::microseconds(2000));
      }
      RCLCPP_INFO(this->get_logger(), "Here6\n");
      continue;
    }
    for (i = 0; i < pack_num; i++) {
      RCLCPP_INFO(this->get_logger(), "Here7\n");
      if(!mag_ptr){
        RCLCPP_INFO(this->get_logger(), "Problem idenified\n");
      }else{
        sensors_event_t data_mag;
        data_mag = *mag_ptr;
        RCLCPP_INFO(this->get_logger(), "[sensor_client_test] mag time: %ld x: %f y: %f z: %f mag reserved0: %d",
            data_mag.timestamp, data_mag.magnetic.x, data_mag.magnetic.y, data_mag.magnetic.z, data_mag.reserved0);
      }
      RCLCPP_INFO(this->get_logger(), "Here7.5\n");
      container = std::make_unique<qrb::ros::MagTypeAdapter>(*mag_ptr);
      RCLCPP_INFO(this->get_logger(), "Here8\n");
      if (debug_) {
        time_nanosec = (get_clock()->now()).nanoseconds();
        latency_sum += time_nanosec - mag_ptr->timestamp;
        container->header.stamp = get_clock()->now();
      }
      publisher_->publish(std::move(container));
      mag_ptr += 1; //what
    }
    if (debug_) {
      RCLCPP_INFO(this->get_logger(), "latency: %lld ns", latency_sum / pack_num);
      latency_sum = 0;
    }
  }
}    
}  // namespace ros
}  // namespace qrb

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(qrb::ros::MagComponent)