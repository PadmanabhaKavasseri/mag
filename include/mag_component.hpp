#include "rclcpp/rclcpp.hpp"
#include "sensor_client.h"
#include "std_msgs/msg/string.hpp"

#define TOPIC_NAME "imu"
// #define TOPIC_TYPE qrb::ros::MagTypeAdapter
#define RETRY_MAX 30
#define RETRY_INTERVAL 2  // 2s


class MagComponent: public rclcpp::Node
{
public:
  bool is_working;
  MagComponent();
  // ~MagComponent();
  
private:
  // bool init();
  // void connect_success();
  // void retry_conection();
  // void publish_msg();

  void test();
  SensorClient sensor_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

