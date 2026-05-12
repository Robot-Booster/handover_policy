#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "camera/preprocess_pipeline.hpp"

namespace camera {

class CameraPreprocessNode : public rclcpp::Node {
public:
  explicit CameraPreprocessNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void onImage(sensor_msgs::msg::Image::ConstSharedPtr msg);

  static std::string resolveTopic(const std::string & raw, const std::string & default_value);
  void logResolvedParameters() const;

  PreprocessPipeline pipeline_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_cache_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string image_rect_topic_;
  std::string camera_info_rect_topic_;
};

}  // namespace camera
