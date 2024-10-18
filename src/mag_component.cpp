#include "/home/rosws/src/qrb_ros_mag/include/mag_component.hpp"

#include <rclcpp/rclcpp.hpp>


MagComponent::MagComponent() : Node("minimal_publisher")
{
    int i = 0;
    if(!sensor_client_.CreateConnection()){
        RCLCPP_ERROR(this->get_logger(), "mag client connect failed.");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "imu client connect success");
    }
    RCLCPP_INFO(this->get_logger(), "imu client connect success");
    sensors_event_t* mag_ptr;
    int32_t pack_num = 0;
    //why have a double pointer here and not just pass mag_ptr?
    sensor_client_.GetMagData(&mag_ptr, &pack_num);
    sensors_event_t data_mag;
    data_mag = *mag_ptr;
    mag_ptr += 1;
    RCLCPP_INFO(this->get_logger(), "[sensor_client_test] mag time: %ld x: %f y: %f z: %f mag reserved0: %d",
            data_mag.timestamp, data_mag.magnetic.x, data_mag.magnetic.y, data_mag.magnetic.z, data_mag.reserved0);

    while(1){
        bool ret_mag = sensor_client_.GetMagData(&mag_ptr, &pack_num);
            if (!ret_mag) {
                std::this_thread::sleep_for(std::chrono::microseconds(5 * 1000));
                continue;
            }
            for (int i = 0; i < pack_num; i++) {
                data_mag = *mag_ptr;
                mag_ptr += 1;
                RCLCPP_INFO(this->get_logger(), "[sensor_client_test] mag time: %ld x: %f y: %f z: %f mag reserved0: %d",
                    data_mag.timestamp, data_mag.magnetic.x, data_mag.magnetic.y, data_mag.magnetic.z, data_mag.reserved0);

            }
    }
    


}   

void MagComponent::test()
{
    RCLCPP_INFO(this->get_logger(), "Hello from Mag Node\n");
}



    
int main(int argc, char **argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of MagComponent
    auto mag = std::make_shared<MagComponent>();

    // Spin the node
    rclcpp::spin(mag);

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}
