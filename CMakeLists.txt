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

add_executable(mag_component src/mag_component.cpp)
ament_target_dependencies(mag_component rclcpp std_msgs)

target_link_libraries(mag_component
    sensor_client
)

install(TARGETS
  mag_component
  DESTINATION lib/${PROJECT_NAME})

ament_package()
