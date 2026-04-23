#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pc_processor
{

class DepthToCloudNode : public rclcpp::Node
{
public:
  DepthToCloudNode();

private:
  void onMask(const sensor_msgs::msg::Image::SharedPtr msg);
  void onMaskCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void processDepthFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);

  std::pair<std::vector<float>, std::string> align(
    const std::vector<float> & xyz,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

  std::optional<std::vector<float>> applyMask(
    std::vector<float> xyz,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const std::string & cloud_frame);

  void publishCloud(
    const std::vector<float> & xyz,
    const std::vector<uint8_t> & rgb,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const std::string & frame_id);

  void logParameters();
  void _loadRoiMask();
  void _applyRoiMask(sensor_msgs::msg::Image & depth_img) const;
  void _validateMaskCameraInfoOnce(const sensor_msgs::msg::CameraInfo & camera_info_msg);
  bool _buildMaskForApply(
    const sensor_msgs::msg::Image & raw_mask,
    sensor_msgs::msg::Image & out_mask,
    std::string & err_msg);

  std::string depth_topic_;
  std::string depth_camera_info_topic_;
  std::string mask_topic_;
  std::string mask_camera_info_topic_;
  std::string cloud_topic_;
  std::string align_frame_;
  std::string roi_mask_image_path_;
  double heatmap_min_depth_m_;
  double heatmap_max_depth_m_;
  bool undistort_mask_enabled_{true};
  std::vector<uint8_t> roi_mask_;
  uint32_t roi_mask_width_{0};
  uint32_t roi_mask_height_{0};
  bool roi_mask_ready_{false};
  bool mask_camera_info_checked_{false};
  bool mask_distortion_zero_warned_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr mask_camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  sensor_msgs::msg::Image::SharedPtr latest_mask_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_mask_camera_info_;

  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace pc_processor
