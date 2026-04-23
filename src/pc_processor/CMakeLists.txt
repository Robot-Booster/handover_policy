cmake_minimum_required(VERSION 3.8)
project(pc_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs imgproc calib3d)

add_library(pc_processor_utils src/pc_processor_utils.cpp)
target_include_directories(pc_processor_utils PUBLIC include)
ament_target_dependencies(
  pc_processor_utils
  sensor_msgs
  tf2
  geometry_msgs
  tf2_geometry_msgs
)
target_link_libraries(pc_processor_utils ${OpenCV_LIBRARIES})

add_library(depth_to_cloud_node_lib src/depth_to_cloud_node.cpp)
target_include_directories(depth_to_cloud_node_lib PUBLIC include)
ament_target_dependencies(
  depth_to_cloud_node_lib
  rclcpp
  sensor_msgs
  std_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(depth_to_cloud_node_lib pc_processor_utils)

add_executable(depth_to_cloud_node src/main.cpp)
target_link_libraries(depth_to_cloud_node depth_to_cloud_node_lib)
ament_target_dependencies(depth_to_cloud_node rclcpp)

install(
  TARGETS pc_processor_utils depth_to_cloud_node_lib depth_to_cloud_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
