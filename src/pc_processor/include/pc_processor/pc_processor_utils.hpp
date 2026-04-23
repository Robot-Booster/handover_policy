#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace pc_processor
{

std::vector<float> projectDepthToXyz(
  const sensor_msgs::msg::Image & depth_msg,
  const sensor_msgs::msg::CameraInfo & camera_info_msg);

std::vector<float> alignXyzToFrame(
  const std::vector<float> & xyz,
  const geometry_msgs::msg::TransformStamped & transform);

void applyMono8Mask(
  std::vector<float> & xyz,
  const sensor_msgs::msg::Image & mask_msg);
void applyMono8MaskByProjection(
  std::vector<float> & xyz,
  const sensor_msgs::msg::Image & mask_msg,
  const sensor_msgs::msg::CameraInfo & camera_info_msg);

std::vector<uint8_t> colorizeDepthHeatmap(
  const std::vector<float> & xyz,
  float min_depth_m,
  float max_depth_m);

sensor_msgs::msg::PointCloud2 buildXyzrgbPointCloud(
  const std::vector<float> & xyz,
  const std::vector<uint8_t> & rgb,
  const std_msgs::msg::Header & header);

bool isDepth32FC1Tight(const sensor_msgs::msg::Image & msg);
bool isMaskMono8Tight(const sensor_msgs::msg::Image & msg);
bool isMaskShapeMatch(const sensor_msgs::msg::Image & mask_msg, const sensor_msgs::msg::Image & depth_msg);
bool undistortMono8Mask(
  const sensor_msgs::msg::Image & in_mask_msg,
  const sensor_msgs::msg::CameraInfo & camera_info_msg,
  sensor_msgs::msg::Image & out_mask_msg,
  std::string & err_msg);
bool loadRoiMaskMono8(
  const std::string & path,
  std::vector<uint8_t> & out_mask,
  uint32_t & out_width,
  uint32_t & out_height,
  std::string & err_msg);
void applyRoiMaskToDepth32FC1(
  sensor_msgs::msg::Image & depth_msg,
  const std::vector<uint8_t> & roi_mask,
  uint32_t roi_width,
  uint32_t roi_height,
  bool & size_match);

}  // namespace pc_processor
