cmake_minimum_required(VERSION 3.8)
project(server_client_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)


add_executable(simple_servernode src/server.cpp)
ament_target_dependencies(simple_servernode rclcpp std_msgs example_interfaces)

add_executable(simple_clientnode src/client.cpp)
ament_target_dependencies(simple_clientnode rclcpp std_msgs example_interfaces)
install( TARGETS
  simple_servernode
  simple_clientnode
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
