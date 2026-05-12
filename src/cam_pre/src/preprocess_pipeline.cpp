#include "camera/preprocess_pipeline.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

namespace camera {

namespace {

bool fillMatrices(const sensor_msgs::msg::CameraInfo & info, cv::Mat & K, cv::Mat & D)
{
  if (info.width <= 0 || info.height <= 0) {
    return false;
  }
  if (info.k.size() < 9 || std::abs(info.k[0]) < 1e-12) {
    return false;
  }

  K = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) {
    K.at<double>(i / 3, i % 3) = info.k[i];
  }

  const auto nd = info.d.size();
  if (nd == 0) {
    D = cv::Mat();
  } else {
    D = cv::Mat(static_cast<int>(nd), 1, CV_64F);
    for (size_t i = 0; i < nd; ++i) {
      D.at<double>(static_cast<int>(i)) = info.d[i];
    }
  }
  return true;
}

}  // namespace

void PreprocessPipeline::updateCalibration(const sensor_msgs::msg::CameraInfo & info)
{
  maps_ready_ = false;
  map1_.release();
  map2_.release();
  calibrated_size_ = {0, 0};
  cv::Mat K;
  cv::Mat D;
  if (!fillMatrices(info, K, D)) {
    return;
  }

  const cv::Size size(info.width, info.height);
  calibrated_size_ = size;
  const cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, D, size, /*alpha=*/1.0, size);
  new_camera_matrix_ = new_K.clone();

  cv::initUndistortRectifyMap(
    K, D, cv::Mat(), new_K, size, CV_16SC2, map1_, map2_);

  maps_ready_ = true;
}

bool PreprocessPipeline::runUndistort(const cv::Mat & in_image, cv::Mat & out_image) const
{
  if (!maps_ready_ || map1_.empty()) {
    return false;
  }
  if (in_image.empty()) {
    return false;
  }
  if (in_image.size() != calibrated_size_) {
    return false;
  }
  cv::remap(in_image, out_image, map1_, map2_, cv::INTER_LINEAR);
  return true;
}

void PreprocessPipeline::buildRectifiedCameraInfo(
  const sensor_msgs::msg::CameraInfo & src,
  sensor_msgs::msg::CameraInfo & out,
  const cv::Size & image_size) const
{
  out = src;
  out.height = static_cast<uint32_t>(image_size.height);
  out.width = static_cast<uint32_t>(image_size.width);
  out.d.clear();

  for (int i = 0; i < 9; ++i) {
    out.k[i] = new_camera_matrix_.at<double>(i / 3, i % 3);
  }

  out.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  const double fx = new_camera_matrix_.at<double>(0, 0);
  const double fy = new_camera_matrix_.at<double>(1, 1);
  const double cx = new_camera_matrix_.at<double>(0, 2);
  const double cy = new_camera_matrix_.at<double>(1, 2);

  out.p = {
    fx, 0.0, cx, 0.0,
    0.0, fy, cy, 0.0,
    0.0, 0.0, 1.0, 0.0};

  out.distortion_model = src.distortion_model;

  out.roi.x_offset = 0;
  out.roi.y_offset = 0;
  out.roi.height = static_cast<uint32_t>(image_size.height);
  out.roi.width = static_cast<uint32_t>(image_size.width);
  out.roi.do_rectify = false;
}

}  // namespace camera
