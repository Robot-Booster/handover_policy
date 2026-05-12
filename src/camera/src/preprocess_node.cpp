#include "camera/preprocess_node.hpp"

#include <opencv2/core.hpp>

#include <cv_bridge/cv_bridge.h>

namespace camera {

namespace {

constexpr char kDefaultImageTopic[] = "~/image_raw";
constexpr char kDefaultCameraInfoTopic[] = "~/camera_info";
constexpr char kDefaultImageRectTopic[] = "~/image_rect";
constexpr char kDefaultCameraInfoRectTopic[] = "~/camera_info_rect";

std::string trim(std::string s)
{
  const char * ws = " \t\n\r";
  const auto b = s.find_first_not_of(ws);
  if (b == std::string::npos) {
    return {};
  }
  const auto e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

}  // namespace

std::string PreprocessNode::resolveTopic(const std::string & raw, const std::string & default_value)
{
  const std::string t = trim(raw);
  return t.empty() ? default_value : t;
}

PreprocessNode::PreprocessNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("preprocess_node", options)
{
  declare_parameter<std::string>("image_topic", kDefaultImageTopic);
  declare_parameter<std::string>("camera_info_topic", kDefaultCameraInfoTopic);
  declare_parameter<std::string>("image_rect_topic", kDefaultImageRectTopic);
  declare_parameter<std::string>("camera_info_rect_topic", kDefaultCameraInfoRectTopic);

  image_topic_ = resolveTopic(get_parameter("image_topic").as_string(), kDefaultImageTopic);
  camera_info_topic_ =
    resolveTopic(get_parameter("camera_info_topic").as_string(), kDefaultCameraInfoTopic);
  image_rect_topic_ =
    resolveTopic(get_parameter("image_rect_topic").as_string(), kDefaultImageRectTopic);
  camera_info_rect_topic_ = resolveTopic(
    get_parameter("camera_info_rect_topic").as_string(), kDefaultCameraInfoRectTopic);

  logResolvedParameters();

  const rclcpp::QoS qos(1);
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_,
    qos,
    std::bind(&PreprocessNode::onCameraInfo, this, std::placeholders::_1));

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_,
    qos,
    std::bind(&PreprocessNode::onImage, this, std::placeholders::_1));

  image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_rect_topic_, qos);
  camera_info_pub_ =
    create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_rect_topic_, qos);
}

void PreprocessNode::logResolvedParameters() const
{
  RCLCPP_INFO(get_logger(), "image_topic=%s", image_topic_.c_str());
  RCLCPP_INFO(get_logger(), "camera_info_topic=%s", camera_info_topic_.c_str());
  RCLCPP_INFO(get_logger(), "image_rect_topic=%s", image_rect_topic_.c_str());
  RCLCPP_INFO(get_logger(), "camera_info_rect_topic=%s", camera_info_rect_topic_.c_str());
}

void PreprocessNode::onCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  camera_info_cache_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg);
  pipeline_.updateCalibration(*msg);
  if (!pipeline_.calibrationReady()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Invalid CameraInfo; undistort maps not updated.");
  }
}

void PreprocessNode::onImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (!camera_info_cache_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "No CameraInfo received yet; skipping frame.");
    return;
  }
  if (!pipeline_.calibrationReady()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Calibration not ready; skipping frame.");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "cv_bridge failed: %s", e.what());
    return;
  }

  cv::Mat out;
  if (!pipeline_.runUndistort(cv_ptr->image, out)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Undistort failed (image size vs calibration or maps).");
    return;
  }

  cv_bridge::CvImage bridge(msg->header, msg->encoding, out);
  sensor_msgs::msg::Image out_img;
  bridge.toImageMsg(out_img);

  sensor_msgs::msg::CameraInfo out_info;
  pipeline_.buildRectifiedCameraInfo(*camera_info_cache_, out_info, out.size());
  out_info.header = msg->header;

  image_pub_->publish(out_img);
  camera_info_pub_->publish(out_info);
}

}  // namespace camera
