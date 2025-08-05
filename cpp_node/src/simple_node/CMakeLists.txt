cmake_minimum_required(VERSION 3.8)
project(simple_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_node src/simple_node.cpp)

ament_target_dependencies(simple_node rclcpp std_msgs)

add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)
 
add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

add_executable(simple_pub_sub src/simple_pub_sub.cpp)
ament_target_dependencies(simple_pub_sub rclcpp std_msgs)

install( TARGETS
  simple_node
  simple_publisher
  simple_subscriber
  simple_pub_sub
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
