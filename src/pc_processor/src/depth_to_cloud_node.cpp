#include "pc_processor/depth_to_cloud_node.hpp"

#include <algorithm>
#include <functional>
#include <utility>

#include "pc_processor/pc_processor_utils.hpp"
#include <stdexcept>
#include <tf2/exceptions.h>

namespace pc_processor
{

DepthToCloudNode::DepthToCloudNode()
: Node("depth_to_cloud_node"),
  depth_sub_(this, ""),
  camera_info_sub_(this, ""),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  this->declare_parameter<std::string>("depth_topic", "~/depth");
  this->declare_parameter<std::string>("depth_camera_info_topic", "~/depth_camera_info");
  this->declare_parameter<std::string>("mask_topic", "");
  this->declare_parameter<std::string>("mask_camera_info_topic", "");
  this->declare_parameter<std::string>("cloud_topic", "~/pointcloud");
  this->declare_parameter<std::string>("align_frame", "");
  this->declare_parameter<std::string>("roi_mask_image_path", "");
  this->declare_parameter<double>("heatmap_min_depth_m", 0.1);
  this->declare_parameter<double>("heatmap_max_depth_m", 2.0);
  this->declare_parameter<bool>("undistort_mask_enabled", true);

  depth_topic_ = this->get_parameter("depth_topic").as_string();
  depth_camera_info_topic_ = this->get_parameter("depth_camera_info_topic").as_string();
  mask_topic_ = this->get_parameter("mask_topic").as_string();
  mask_camera_info_topic_ = this->get_parameter("mask_camera_info_topic").as_string();
  cloud_topic_ = this->get_parameter("cloud_topic").as_string();
  align_frame_ = this->get_parameter("align_frame").as_string();
  roi_mask_image_path_ = this->get_parameter("roi_mask_image_path").as_string();
  heatmap_min_depth_m_ = this->get_parameter("heatmap_min_depth_m").as_double();
  heatmap_max_depth_m_ = this->get_parameter("heatmap_max_depth_m").as_double();
  undistort_mask_enabled_ = this->get_parameter("undistort_mask_enabled").as_bool();

  if (undistort_mask_enabled_ && mask_camera_info_topic_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "undistort_mask_enabled=true but mask_camera_info_topic is empty.");
    throw std::runtime_error("mask_camera_info_topic is required when undistort_mask_enabled=true");
  }

  _loadRoiMask();

  logParameters();

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 5);
  if (!mask_topic_.empty()) {
    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      mask_topic_, 5, std::bind(&DepthToCloudNode::onMask, this, std::placeholders::_1));
  }
  if (!mask_camera_info_topic_.empty()) {
    mask_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      mask_camera_info_topic_, 5, std::bind(&DepthToCloudNode::onMaskCameraInfo, this, std::placeholders::_1));
  }

  depth_sub_.subscribe(this, depth_topic_);
  camera_info_sub_.subscribe(this, depth_camera_info_topic_);
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), depth_sub_, camera_info_sub_);
  sync_->registerCallback(std::bind(
    &DepthToCloudNode::processDepthFrame, this, std::placeholders::_1, std::placeholders::_2));
}

void DepthToCloudNode::onMask(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_mask_ = msg;
}

void DepthToCloudNode::onMaskCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (undistort_mask_enabled_) {
    _validateMaskCameraInfoOnce(*msg);
  }
  latest_mask_camera_info_ = msg;
}

void DepthToCloudNode::processDepthFrame(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  if (!isDepth32FC1Tight(*depth_msg)) {
    RCLCPP_WARN(this->get_logger(), "Depth image must be 32FC1 with tight step.");
    return;
  }

  sensor_msgs::msg::Image depth_for_process = *depth_msg;
  _applyRoiMask(depth_for_process);

  // 主数据流 Main data flow: roi -> project -> align -> mask -> colorize -> publish.
  auto xyz = projectDepthToXyz(depth_for_process, *camera_info_msg);
  auto aligned = align(xyz, depth_msg);
  auto masked = applyMask(std::move(aligned.first), depth_msg, aligned.second);
  if (!masked.has_value()) {
    return;
  }
  xyz = std::move(masked.value());
  auto rgb = colorizeDepthHeatmap(
    xyz,
    static_cast<float>(heatmap_min_depth_m_),
    static_cast<float>(heatmap_max_depth_m_));
  publishCloud(xyz, rgb, depth_msg, aligned.second);
}

std::pair<std::vector<float>, std::string> DepthToCloudNode::align(
  const std::vector<float> & xyz,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{
  const std::string source_frame = depth_msg->header.frame_id;
  const std::string target_frame = align_frame_.empty() ? source_frame : align_frame_;
  if (target_frame == source_frame) {
    return {xyz, target_frame};
  }

  try {
    auto tf_msg = tf_buffer_.lookupTransform(
      target_frame, source_frame, depth_msg->header.stamp, tf2::durationFromSec(0.05));
    return {alignXyzToFrame(xyz, tf_msg), target_frame};
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF align failed: %s", ex.what());
    return {xyz, source_frame};
  }
}

std::optional<std::vector<float>> DepthToCloudNode::applyMask(
  std::vector<float> xyz,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const std::string & cloud_frame)
{
  if (mask_topic_.empty() || !latest_mask_) {
    return std::make_optional(std::move(xyz));
  }
  if (!isMaskMono8Tight(*latest_mask_)) {
    RCLCPP_WARN(this->get_logger(), "Mask image must be mono8 with tight step.");
    return std::make_optional(std::move(xyz));
  }

  sensor_msgs::msg::Image mask_for_apply;
  std::string mask_build_err;
  if (!_buildMaskForApply(*latest_mask_, mask_for_apply, mask_build_err)) {
    RCLCPP_ERROR(this->get_logger(), "Build mask failed: %s", mask_build_err.c_str());
    return std::nullopt;
  }

  // 掩码数据流：先按开关决定是否去畸变，再进入现有掩码流程。
  const auto & mask_for_apply_ref = mask_for_apply;

  if (mask_camera_info_topic_.empty()) {
    if (mask_for_apply_ref.header.frame_id != cloud_frame) {
      RCLCPP_WARN(
        this->get_logger(),
        "Mask frame mismatch with cloud frame. Skip mask for this frame.");
      return std::make_optional(std::move(xyz));
    }
    if (!isMaskShapeMatch(mask_for_apply_ref, *depth_msg)) {
      RCLCPP_WARN(this->get_logger(), "Mask shape mismatch. Skip mask for this frame.");
      return std::make_optional(std::move(xyz));
    }
    applyMono8Mask(xyz, mask_for_apply_ref);
    return std::make_optional(std::move(xyz));
  }

  if (!latest_mask_camera_info_) {
    RCLCPP_ERROR(this->get_logger(), "mask_camera_info_topic is set but camera info not received.");
    return std::nullopt;
  }

  if (latest_mask_camera_info_->header.frame_id != cloud_frame) {
    RCLCPP_WARN(
      this->get_logger(),
      "Mask camera info frame mismatch with final cloud frame. Fallback to index mask mode.");
    if (mask_for_apply_ref.header.frame_id != cloud_frame) {
      RCLCPP_WARN(
        this->get_logger(),
        "Mask frame mismatch with cloud frame in fallback index mode. Skip mask.");
      return std::make_optional(std::move(xyz));
    }
    if (!isMaskShapeMatch(mask_for_apply_ref, *depth_msg)) {
      RCLCPP_WARN(this->get_logger(), "Mask shape mismatch for fallback index mode. Skip mask.");
      return std::make_optional(std::move(xyz));
    }
    applyMono8Mask(xyz, mask_for_apply_ref);
    return std::make_optional(std::move(xyz));
  }

  applyMono8MaskByProjection(xyz, mask_for_apply_ref, *latest_mask_camera_info_);
  return std::make_optional(std::move(xyz));
}

void DepthToCloudNode::publishCloud(
  const std::vector<float> & xyz,
  const std::vector<uint8_t> & rgb,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const std::string & frame_id)
{
  auto header = depth_msg->header;
  header.frame_id = frame_id;
  auto cloud = buildXyzrgbPointCloud(xyz, rgb, header);
  cloud_pub_->publish(cloud);
}

void DepthToCloudNode::logParameters()
{
  RCLCPP_INFO(this->get_logger(), "depth_topic=%s", depth_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "depth_camera_info_topic=%s", depth_camera_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "mask_topic=%s", mask_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "mask_camera_info_topic=%s", mask_camera_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "cloud_topic=%s", cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "align_frame=%s", align_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "roi_mask_image_path=%s", roi_mask_image_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "heatmap_min_depth_m=%.3f", heatmap_min_depth_m_);
  RCLCPP_INFO(this->get_logger(), "heatmap_max_depth_m=%.3f", heatmap_max_depth_m_);
  RCLCPP_INFO(this->get_logger(), "undistort_mask_enabled=%s", undistort_mask_enabled_ ? "true" : "false");
}

void DepthToCloudNode::_validateMaskCameraInfoOnce(
  const sensor_msgs::msg::CameraInfo & camera_info_msg)
{
  if (mask_camera_info_checked_) {
    return;
  }

  if (camera_info_msg.distortion_model != "plumb_bob") {
    RCLCPP_ERROR(
      this->get_logger(),
      "Only plumb_bob distortion model is supported for mask undistort, got: %s",
      camera_info_msg.distortion_model.c_str());
    throw std::runtime_error("unsupported mask distortion model");
  }

  const bool all_zero_distortion = std::all_of(
    camera_info_msg.d.begin(), camera_info_msg.d.end(), [](double v) { return v == 0.0; });
  if (all_zero_distortion && !mask_distortion_zero_warned_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Mask camera distortion coefficients are all zero, mask camera is near distortion-free.");
    mask_distortion_zero_warned_ = true;
  }

  mask_camera_info_checked_ = true;
}

bool DepthToCloudNode::_buildMaskForApply(
  const sensor_msgs::msg::Image & raw_mask,
  sensor_msgs::msg::Image & out_mask,
  std::string & err_msg)
{
  if (!undistort_mask_enabled_) {
    out_mask = raw_mask;
    err_msg.clear();
    return true;
  }

  if (!latest_mask_camera_info_) {
    err_msg = "undistort_mask_enabled=true but mask camera info not received";
    return false;
  }

  _validateMaskCameraInfoOnce(*latest_mask_camera_info_);
  if (!undistortMono8Mask(raw_mask, *latest_mask_camera_info_, out_mask, err_msg)) {
    return false;
  }
  return true;
}

void DepthToCloudNode::_loadRoiMask()
{
  if (roi_mask_image_path_.empty()) {
    roi_mask_ready_ = false;
    return;
  }

  std::string err_msg;
  roi_mask_ready_ = loadRoiMaskMono8(
    roi_mask_image_path_, roi_mask_, roi_mask_width_, roi_mask_height_, err_msg);
  if (!roi_mask_ready_) {
    RCLCPP_ERROR(this->get_logger(), "Load ROI mask failed: %s", err_msg.c_str());
    throw std::runtime_error("roi_mask_image_path is set but ROI image load failed");
  }
}

void DepthToCloudNode::_applyRoiMask(sensor_msgs::msg::Image & depth_img) const
{
  if (!roi_mask_ready_) {
    return;
  }

  bool size_match = false;
  applyRoiMaskToDepth32FC1(depth_img, roi_mask_, roi_mask_width_, roi_mask_height_, size_match);
  if (!size_match) {
    RCLCPP_WARN(this->get_logger(), "ROI size mismatch, skip ROI for this frame.");
  }
}

}  // namespace pc_processor
