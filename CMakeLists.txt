cmake_minimum_required(VERSION 3.5)
project(qrb_ros_mag)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(./include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(rclcpp_components REQUIRED)

add_library(mag_component src/mag_component.cpp src/mag_type_adapter.cpp)
ament_target_dependencies(mag_component rclcpp std_msgs sensor_msgs rclcpp_components)

target_link_libraries(mag_component
    sensor_client
)

add_executable(mag_node src/mag_node.cpp src/mag_type_adapter.cpp)
ament_target_dependencies(mag_node rclcpp std_msgs sensor_msgs rclcpp_components)

target_link_libraries(mag_node
    sensor_client
    mag_component
)

install(TARGETS
  mag_component
  mag_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
