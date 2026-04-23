#include "pc_processor/depth_to_cloud_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pc_processor::DepthToCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
