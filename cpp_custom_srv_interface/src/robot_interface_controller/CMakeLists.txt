cmake_minimum_required(VERSION 3.8)
project(robot_interface_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_interface_publisher src/custom_publisher.cpp) 
ament_target_dependencies(robot_interface_publisher rclcpp robot_interfaces std_msgs)

add_executable(robot_interface_subscriber src/custom_subscriber.cpp) 
ament_target_dependencies(robot_interface_subscriber rclcpp robot_interfaces)

add_executable(robot_interface_system_sim src/custom_simulate_system.cpp) 
ament_target_dependencies(robot_interface_system_sim rclcpp std_msgs robot_interfaces)

include_directories("include")
install(
  TARGETS
  robot_interface_publisher
  robot_interface_subscriber
  robot_interface_system_sim
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
