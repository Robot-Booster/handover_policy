#pragma once

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace camera {

/// Undistort + rectified CameraInfo; extend with more steps later (same file).
class PreprocessPipeline {
public:
  void updateCalibration(const sensor_msgs::msg::CameraInfo & info);
  bool calibrationReady() const { return maps_ready_; }

  bool runUndistort(const cv::Mat & in_image, cv::Mat & out_image) const;

  void buildRectifiedCameraInfo(
    const sensor_msgs::msg::CameraInfo & src,
    sensor_msgs::msg::CameraInfo & out,
    const cv::Size & image_size) const;

private:
  cv::Mat map1_;
  cv::Mat map2_;
  cv::Mat new_camera_matrix_;  // 3x3 CV_64F
  cv::Size calibrated_size_{0, 0};
  bool maps_ready_{false};
};

}  // namespace camera
