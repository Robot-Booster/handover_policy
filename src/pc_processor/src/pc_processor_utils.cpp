#include "pc_processor/pc_processor_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pc_processor
{

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
}  // namespace

std::vector<float> projectDepthToXyz(
  const sensor_msgs::msg::Image & depth_msg,
  const sensor_msgs::msg::CameraInfo & camera_info_msg)
{
  const auto width = depth_msg.width;
  const auto height = depth_msg.height;
  const auto * depth = reinterpret_cast<const float *>(depth_msg.data.data());
  const float fx = static_cast<float>(camera_info_msg.k[0]);
  const float fy = static_cast<float>(camera_info_msg.k[4]);
  const float cx = static_cast<float>(camera_info_msg.k[2]);
  const float cy = static_cast<float>(camera_info_msg.k[5]);

  std::vector<float> xyz(width * height * 3, kNaN);
  for (uint32_t v = 0; v < height; ++v) {
    for (uint32_t u = 0; u < width; ++u) {
      const size_t idx = static_cast<size_t>(v) * width + u;
      const float z = depth[idx];
      if (!std::isfinite(z) || z <= 0.0f) {
        continue;
      }
      xyz[idx * 3 + 0] = (static_cast<float>(u) - cx) * z / fx;
      xyz[idx * 3 + 1] = (static_cast<float>(v) - cy) * z / fy;
      xyz[idx * 3 + 2] = z;
    }
  }
  return xyz;
}

std::vector<float> alignXyzToFrame(
  const std::vector<float> & xyz,
  const geometry_msgs::msg::TransformStamped & transform)
{
  std::vector<float> out(xyz.size(), kNaN);
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);
  tf2::Matrix3x3 rot(q);
  const auto & t = transform.transform.translation;

  for (size_t i = 0; i + 2 < xyz.size(); i += 3) {
    const float z = xyz[i + 2];
    if (!std::isfinite(z) || z <= 0.0f) {
      continue;
    }
    tf2::Vector3 p(xyz[i + 0], xyz[i + 1], xyz[i + 2]);
    tf2::Vector3 p_aligned = rot * p + tf2::Vector3(t.x, t.y, t.z);
    out[i + 0] = static_cast<float>(p_aligned.x());
    out[i + 1] = static_cast<float>(p_aligned.y());
    out[i + 2] = static_cast<float>(p_aligned.z());
  }
  return out;
}

void applyMono8Mask(std::vector<float> & xyz, const sensor_msgs::msg::Image & mask_msg)
{
  const auto * mask = reinterpret_cast<const uint8_t *>(mask_msg.data.data());
  const size_t pixel_count = static_cast<size_t>(mask_msg.width) * mask_msg.height;
  for (size_t i = 0; i < pixel_count; ++i) {
    if (mask[i] > 0) {
      continue;
    }
    xyz[i * 3 + 0] = kNaN;
    xyz[i * 3 + 1] = kNaN;
    xyz[i * 3 + 2] = kNaN;
  }
}

void applyMono8MaskByProjection(
  std::vector<float> & xyz,
  const sensor_msgs::msg::Image & mask_msg,
  const sensor_msgs::msg::CameraInfo & camera_info_msg)
{
  const auto * mask = reinterpret_cast<const uint8_t *>(mask_msg.data.data());
  const float fx = static_cast<float>(camera_info_msg.k[0]);
  const float fy = static_cast<float>(camera_info_msg.k[4]);
  const float cx = static_cast<float>(camera_info_msg.k[2]);
  const float cy = static_cast<float>(camera_info_msg.k[5]);
  const int width = static_cast<int>(mask_msg.width);
  const int height = static_cast<int>(mask_msg.height);

  for (size_t i = 0; i + 2 < xyz.size(); i += 3) {
    const float x = xyz[i + 0];
    const float y = xyz[i + 1];
    const float z = xyz[i + 2];
    if (!std::isfinite(z) || z <= 0.0f) {
      continue;
    }

    const int u = static_cast<int>(std::lround((x / z) * fx + cx));
    const int v = static_cast<int>(std::lround((y / z) * fy + cy));
    if (u < 0 || v < 0 || u >= width || v >= height) {
      xyz[i + 0] = kNaN;
      xyz[i + 1] = kNaN;
      xyz[i + 2] = kNaN;
      continue;
    }

    const size_t mask_idx = static_cast<size_t>(v) * static_cast<size_t>(width) + static_cast<size_t>(u);
    if (mask[mask_idx] == 0) {
      xyz[i + 0] = kNaN;
      xyz[i + 1] = kNaN;
      xyz[i + 2] = kNaN;
    }
  }
}

std::vector<uint8_t> colorizeDepthHeatmap(
  const std::vector<float> & xyz,
  float min_depth_m,
  float max_depth_m)
{
  std::vector<uint8_t> rgb((xyz.size() / 3) * 3, 0);
  const float denom = std::max(max_depth_m - min_depth_m, 1e-6f);
  for (size_t i = 0, j = 0; i + 2 < xyz.size(); i += 3, j += 3) {
    const float z = xyz[i + 2];
    if (!std::isfinite(z) || z <= 0.0f) {
      continue;
    }
    float norm = (z - min_depth_m) / denom;
    norm = std::clamp(norm, 0.0f, 1.0f);
    const float inv = 1.0f - norm;

    // 深度着色 Depth colorization: near red, far blue.
    rgb[j + 0] = static_cast<uint8_t>(inv * 255.0f);
    rgb[j + 1] = static_cast<uint8_t>((1.0f - std::fabs(inv - 0.5f) * 2.0f) * 255.0f);
    rgb[j + 2] = static_cast<uint8_t>(norm * 255.0f);
  }
  return rgb;
}

sensor_msgs::msg::PointCloud2 buildXyzrgbPointCloud(
  const std::vector<float> & xyz,
  const std::vector<uint8_t> & rgb,
  const std_msgs::msg::Header & header)
{
  size_t valid_count = 0;
  for (size_t i = 0; i + 2 < xyz.size(); i += 3) {
    const float z = xyz[i + 2];
    if (std::isfinite(z) && z > 0.0f) {
      ++valid_count;
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(valid_count);
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(valid_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud, "rgb");

  for (size_t i = 0, j = 0; i + 2 < xyz.size(); i += 3, j += 3) {
    const float z = xyz[i + 2];
    if (!std::isfinite(z) || z <= 0.0f) {
      continue;
    }
    *iter_x = xyz[i + 0];
    *iter_y = xyz[i + 1];
    *iter_z = xyz[i + 2];
    iter_rgb[0] = rgb[j + 0];
    iter_rgb[1] = rgb[j + 1];
    iter_rgb[2] = rgb[j + 2];
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_rgb;
  }
  return cloud;
}

bool isDepth32FC1Tight(const sensor_msgs::msg::Image & msg)
{
  return msg.encoding == "32FC1" && msg.step == msg.width * 4U;
}

bool isMaskMono8Tight(const sensor_msgs::msg::Image & msg)
{
  return msg.encoding == "mono8" && msg.step == msg.width;
}

bool isMaskShapeMatch(const sensor_msgs::msg::Image & mask_msg, const sensor_msgs::msg::Image & depth_msg)
{
  return mask_msg.width == depth_msg.width && mask_msg.height == depth_msg.height;
}

bool undistortMono8Mask(
  const sensor_msgs::msg::Image & in_mask_msg,
  const sensor_msgs::msg::CameraInfo & camera_info_msg,
  sensor_msgs::msg::Image & out_mask_msg,
  std::string & err_msg)
{
  if (!isMaskMono8Tight(in_mask_msg)) {
    err_msg = "input mask must be mono8 with tight step";
    return false;
  }

  const cv::Mat in_mask(
    static_cast<int>(in_mask_msg.height),
    static_cast<int>(in_mask_msg.width),
    CV_8UC1,
    const_cast<uint8_t *>(in_mask_msg.data.data()));
  if (in_mask.empty()) {
    err_msg = "input mask mat is empty";
    return false;
  }

  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) {
    camera_matrix.at<double>(i / 3, i % 3) = camera_info_msg.k[static_cast<size_t>(i)];
  }
  cv::Mat dist_coeffs(static_cast<int>(camera_info_msg.d.size()), 1, CV_64F);
  for (size_t i = 0; i < camera_info_msg.d.size(); ++i) {
    dist_coeffs.at<double>(static_cast<int>(i), 0) = camera_info_msg.d[i];
  }

  cv::Mat undistorted_mask;
  cv::undistort(in_mask, undistorted_mask, camera_matrix, dist_coeffs);
  if (undistorted_mask.empty() || !undistorted_mask.isContinuous()) {
    err_msg = "undistorted mask mat invalid";
    return false;
  }

  out_mask_msg = in_mask_msg;
  out_mask_msg.width = static_cast<uint32_t>(undistorted_mask.cols);
  out_mask_msg.height = static_cast<uint32_t>(undistorted_mask.rows);
  out_mask_msg.step = out_mask_msg.width;
  out_mask_msg.data.assign(undistorted_mask.data, undistorted_mask.data + undistorted_mask.total());
  err_msg.clear();
  return true;
}

bool loadRoiMaskMono8(
  const std::string & path,
  std::vector<uint8_t> & out_mask,
  uint32_t & out_width,
  uint32_t & out_height,
  std::string & err_msg)
{
  const cv::Mat raw = cv::imread(path, cv::IMREAD_UNCHANGED);
  if (raw.empty()) {
    err_msg = "roi image read failed: " + path;
    return false;
  }

  cv::Mat gray;
  if (raw.channels() == 1) {
    gray = raw;
  } else if (raw.channels() == 3) {
    cv::cvtColor(raw, gray, cv::COLOR_BGR2GRAY);
  } else if (raw.channels() == 4) {
    cv::cvtColor(raw, gray, cv::COLOR_BGRA2GRAY);
  } else {
    err_msg = "unsupported roi channels: " + std::to_string(raw.channels());
    return false;
  }

  if (gray.empty() || !gray.isContinuous()) {
    err_msg = "invalid roi image buffer";
    return false;
  }

  out_width = static_cast<uint32_t>(gray.cols);
  out_height = static_cast<uint32_t>(gray.rows);
  out_mask.assign(gray.data, gray.data + gray.total());
  err_msg.clear();
  return true;
}

void applyRoiMaskToDepth32FC1(
  sensor_msgs::msg::Image & depth_msg,
  const std::vector<uint8_t> & roi_mask,
  uint32_t roi_width,
  uint32_t roi_height,
  bool & size_match)
{
  size_match = (depth_msg.width == roi_width) && (depth_msg.height == roi_height);
  if (!size_match || depth_msg.encoding != "32FC1" || depth_msg.step != depth_msg.width * 4U) {
    return;
  }

  const size_t pixel_count = static_cast<size_t>(depth_msg.width) * depth_msg.height;
  if (roi_mask.size() < pixel_count) {
    size_match = false;
    return;
  }

  auto * depth = reinterpret_cast<float *>(depth_msg.data.data());
  for (size_t i = 0; i < pixel_count; ++i) {
    if (roi_mask[i] == 0) {
      depth[i] = kNaN;
    }
  }
}

}  // namespace pc_processor
