#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "camera/preprocess_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camera::PreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
