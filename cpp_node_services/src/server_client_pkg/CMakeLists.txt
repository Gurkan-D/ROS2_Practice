cmake_minimum_required(VERSION 3.8)
project(server_client_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(simple_server src/server.cpp)
ament_target_dependencies(simple_server rclcpp std_msgs example_interfaces)

add_executable(simple_client src/client.cpp)
ament_target_dependencies(simple_client rclcpp std_msgs example_interfaces)
install( TARGETS
  simple_server
  simple_client
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

