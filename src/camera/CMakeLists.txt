cmake_minimum_required(VERSION 3.8)
project(camera)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)

# Preprocess pipeline — no rclcpp (OpenCV + sensor_msgs only)
add_library(preprocess_pipeline src/preprocess_pipeline.cpp)
target_include_directories(preprocess_pipeline PUBLIC include)
ament_target_dependencies(preprocess_pipeline sensor_msgs)
target_link_libraries(preprocess_pipeline ${OpenCV_LIBS})

add_library(camera_preprocess_node_lib src/camera_preprocess_node.cpp)
target_include_directories(camera_preprocess_node_lib PUBLIC include)
ament_target_dependencies(camera_preprocess_node_lib rclcpp sensor_msgs cv_bridge)
target_link_libraries(camera_preprocess_node_lib preprocess_pipeline)

add_executable(camera_pre src/main.cpp)
target_link_libraries(camera_pre camera_preprocess_node_lib)
ament_target_dependencies(camera_pre rclcpp)

install(
  TARGETS preprocess_pipeline camera_preprocess_node_lib camera_pre
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
