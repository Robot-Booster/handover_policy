#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "camera/camera_preprocess_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camera::CameraPreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
