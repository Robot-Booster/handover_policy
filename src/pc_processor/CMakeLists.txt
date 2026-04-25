cmake_minimum_required(VERSION 3.8)
project(pc_processor)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs imgproc)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenMP REQUIRED)

# 可复用库 pc_pipeline (无 rclcpp 依赖)
add_library(pc_pipeline src/pc_pipeline.cpp)
target_include_directories(pc_pipeline PUBLIC include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(pc_pipeline sensor_msgs)
target_link_libraries(pc_pipeline
  ${OpenCV_LIBRARIES}
  nlohmann_json::nlohmann_json
  OpenMP::OpenMP_CXX
)

# 节点库 perception_node
add_library(perception_node_lib src/perception_node.cpp)
target_include_directories(perception_node_lib PUBLIC include)
ament_target_dependencies(
  perception_node_lib
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(perception_node_lib pc_pipeline OpenMP::OpenMP_CXX)

# 可执行 perception_node
add_executable(perception_node src/main.cpp)
target_link_libraries(perception_node perception_node_lib)
ament_target_dependencies(perception_node rclcpp)

install(
  TARGETS pc_pipeline perception_node_lib perception_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
