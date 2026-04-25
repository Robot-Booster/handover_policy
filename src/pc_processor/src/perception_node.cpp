#include "pc_processor/perception_node.hpp"

#include <chrono>
#include <stdexcept>

#include <omp.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pc_processor
{

PerceptionNode::PerceptionNode()
: Node("perception_node"),
  depth_sub_(this, ""),
  mask_sub_(this, ""),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  _declareAndLoadParams();
  _logParameters();
  _loadStaticResources();

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 5);
  centroid_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(centroid_topic_, 5);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    workspace_marker_topic_, rclcpp::QoS(1).reliable().transient_local());
  marker_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&PerceptionNode::_publishWorkspaceMarker, this));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    depth_camera_info_topic_, 1,
    std::bind(&PerceptionNode::onCameraInfo, this, std::placeholders::_1));

  depth_sub_.subscribe(this, depth_topic_);
  mask_sub_.subscribe(this, seg_mask_topic_);
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), depth_sub_, mask_sub_);
  sync_->setMaxIntervalDuration(
    rclcpp::Duration::from_seconds(depth_mask_max_stamp_diff_sec_));
  sync_->registerCallback(std::bind(
    &PerceptionNode::onSynced, this, std::placeholders::_1, std::placeholders::_2));
}

void PerceptionNode::_declareAndLoadParams()
{
  depth_topic_ = this->declare_parameter<std::string>("depth_topic", "~/depth");
  depth_camera_info_topic_ =
    this->declare_parameter<std::string>("depth_camera_info_topic", "~/depth_camera_info");
  seg_mask_topic_ = this->declare_parameter<std::string>("seg_mask_topic", "~/seg_mask");
  cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "~/pointcloud");
  centroid_topic_ =
    this->declare_parameter<std::string>("centroid_topic", "~/target_centroid");
  workspace_marker_topic_ =
    this->declare_parameter<std::string>("workspace_marker_topic", "~/workspace_marker");
  target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
  roi_mask_image_path_ =
    this->declare_parameter<std::string>("roi_mask_image_path", "");
  workspace_box_json_path_ =
    this->declare_parameter<std::string>("workspace_box_json_path", "");
  depth_mask_max_stamp_diff_sec_ =
    this->declare_parameter<double>("depth_mask_max_stamp_diff_sec", 0.05);
  allow_future_mask_ = this->declare_parameter<bool>("allow_future_mask", false);
  voxel_leaf_size_m_ = static_cast<float>(
    this->declare_parameter<double>("voxel_leaf_size_m", 0.005));
  ror_min_neighbors_ = this->declare_parameter<int>("ror_min_neighbors", 5);
  cluster_min_voxels_ = this->declare_parameter<int>("cluster_min_voxels", 20);
  const auto rgb = this->declare_parameter<std::vector<int64_t>>(
    "output_rgb", std::vector<int64_t>{255, 0, 0});
  if (rgb.size() != 3) {
    throw std::runtime_error("output_rgb must have 3 entries");
  }
  output_r_ = static_cast<uint8_t>(rgb[0]);
  output_g_ = static_cast<uint8_t>(rgb[1]);
  output_b_ = static_cast<uint8_t>(rgb[2]);
  const auto marker_rgba = this->declare_parameter<std::vector<double>>(
    "workspace_marker_rgba", std::vector<double>{0.0, 1.0, 1.0, 0.25});
  if (marker_rgba.size() != 4) {
    throw std::runtime_error("workspace_marker_rgba must have 4 entries");
  }
  marker_r_ = static_cast<float>(marker_rgba[0]);
  marker_g_ = static_cast<float>(marker_rgba[1]);
  marker_b_ = static_cast<float>(marker_rgba[2]);
  marker_a_ = static_cast<float>(marker_rgba[3]);
  if (target_frame_.empty()) {
    throw std::runtime_error("target_frame must not be empty");
  }
}

void PerceptionNode::_logParameters() const
{
  auto L = this->get_logger();
  RCLCPP_INFO(L, "depth_topic=%s", depth_topic_.c_str());
  RCLCPP_INFO(L, "depth_camera_info_topic=%s", depth_camera_info_topic_.c_str());
  RCLCPP_INFO(L, "seg_mask_topic=%s", seg_mask_topic_.c_str());
  RCLCPP_INFO(L, "cloud_topic=%s", cloud_topic_.c_str());
  RCLCPP_INFO(L, "centroid_topic=%s", centroid_topic_.c_str());
  RCLCPP_INFO(L, "workspace_marker_topic=%s", workspace_marker_topic_.c_str());
  RCLCPP_INFO(L, "target_frame=%s", target_frame_.c_str());
  RCLCPP_INFO(L, "workspace_box_frame=%s", target_frame_.c_str());
  RCLCPP_INFO(L, "roi_mask_image_path=%s", roi_mask_image_path_.c_str());
  RCLCPP_INFO(L, "workspace_box_json_path=%s", workspace_box_json_path_.c_str());
  RCLCPP_INFO(L, "depth_mask_max_stamp_diff_sec=%.3f", depth_mask_max_stamp_diff_sec_);
  RCLCPP_INFO(L, "allow_future_mask=%s", allow_future_mask_ ? "true" : "false");
  RCLCPP_INFO(L, "voxel_leaf_size_m=%.4f", voxel_leaf_size_m_);
  RCLCPP_INFO(L, "ror_min_neighbors=%d", ror_min_neighbors_);
  RCLCPP_INFO(L, "cluster_min_voxels=%d", cluster_min_voxels_);
  RCLCPP_INFO(L, "output_rgb=(%u,%u,%u)", output_r_, output_g_, output_b_);
  RCLCPP_INFO(L, "workspace_marker_rgba=(%.2f,%.2f,%.2f,%.2f)", marker_r_, marker_g_, marker_b_, marker_a_);
  RCLCPP_INFO(L, "omp_max_threads=%d", omp_get_max_threads());
}

void PerceptionNode::_loadStaticResources()
{
  if (workspace_box_json_path_.empty()) {
    throw std::runtime_error("workspace_box_json_path must be set");
  }
  obb_box_ = ObbBox::loadFromJson(workspace_box_json_path_);

  if (!roi_mask_image_path_.empty()) {
    roi_mask_ = RoiMask::loadFromFile(roi_mask_image_path_);
    RCLCPP_INFO(
      this->get_logger(), "ROI mask loaded: %ux%u",
      roi_mask_.width(), roi_mask_.height());
  } else {
    RCLCPP_INFO(this->get_logger(), "ROI mask disabled (path empty)");
  }
}

void PerceptionNode::onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (intrinsics_ready_) return;
  intrinsics_ = CameraIntrinsics::fromCameraInfo(*msg);
  intrinsics_ready_ = true;
  RCLCPP_INFO(
    this->get_logger(),
    "Intrinsics cached: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
    intrinsics_.fx, intrinsics_.fy, intrinsics_.cx, intrinsics_.cy);
  _publishWorkspaceMarker();
}

void PerceptionNode::onSynced(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & mask_msg)
{
  if (!intrinsics_ready_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Waiting for camera_info on %s", depth_camera_info_topic_.c_str());
    return;
  }

  // 未来戳保护 Future-stamp guard: drop if mask is newer than depth.
  if (!allow_future_mask_) {
    const rclcpp::Time d(depth_msg->header.stamp);
    const rclcpp::Time m(mask_msg->header.stamp);
    if (m > d) {
      RCLCPP_DEBUG(this->get_logger(), "Drop frame: mask stamp in the future.");
      return;
    }
  }

  processFrame(depth_msg, mask_msg);
}

void PerceptionNode::processFrame(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & mask_msg)
{
  // 1. 融合前置 Pass 1: fused projection
  if (depth_msg->encoding != "32FC1" || depth_msg->step != depth_msg->width * 4U) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Depth must be 32FC1 with tight step. Skip frame.");
    return;
  }
  fusedProject(*depth_msg, intrinsics_, roi_mask_, mask_msg.get(), buffers_.points);
  if (buffers_.points.empty()) return;

  // 2. 先变换到 target_frame，再做工作区裁剪
  if (!_transformAndCropPointsToTarget(
      buffers_.points, depth_msg->header.frame_id, depth_msg->header.stamp))
  {
    return;
  }
  if (buffers_.points.empty()) return;

  // 3. 体素哈希 Pass 3: voxel hashing
  grid_.buildFromPoints(buffers_.points, voxel_leaf_size_m_);

  // 4. ROR + 聚类 + 邻接 Pass 4
  grid_.rorFilter(ror_min_neighbors_);
  grid_.clusterAndAdjacency(cluster_min_voxels_);

  // 5. 选最终 other 簇 Pass 5
  const int ocid = grid_.selectFinalOtherCluster();
  if (ocid < 0) return;
  grid_.extractCluster(ocid, buffers_.final_xyz, buffers_.final_centroid);
  if (buffers_.final_xyz.empty()) return;

  // 6. 直接在 target_frame 发布（不再做末端二次 TF）
  _publishCloud(buffers_.final_xyz, target_frame_, depth_msg->header.stamp);
  _publishCentroid(buffers_.final_centroid, target_frame_, depth_msg->header.stamp);
}

bool PerceptionNode::_transformAndCropPointsToTarget(
  std::vector<Point> & points,
  const std::string & source_frame,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_.lookupTransform(
      target_frame_, source_frame, stamp, tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed for pre-crop: %s", ex.what());
    return false;
  }

  tf2::Quaternion q;
  tf2::fromMsg(tf_msg.transform.rotation, q);
  const tf2::Matrix3x3 m(q);
  const Eigen::Matrix3f R = (Eigen::Matrix3f() <<
    m[0][0], m[0][1], m[0][2],
    m[1][0], m[1][1], m[1][2],
    m[2][0], m[2][1], m[2][2]).finished();
  const Eigen::Vector3f t(
    tf_msg.transform.translation.x,
    tf_msg.transform.translation.y,
    tf_msg.transform.translation.z);

  std::vector<Point> kept;
  kept.reserve(points.size());
  for (const auto & p : points) {
    const Eigen::Vector3f v = R * Eigen::Vector3f(p.x, p.y, p.z) + t;
    if (!obb_box_.contains(v)) {
      continue;
    }
    kept.push_back(Point{v.x(), v.y(), v.z(), p.is_target});
  }
  points.swap(kept);
  return true;
}

void PerceptionNode::_publishCloud(
  const std::vector<Eigen::Vector3f> & xyz,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id = frame_id;
  cloud->header.stamp = stamp;
  cloud->height = 1;
  cloud->width = static_cast<uint32_t>(xyz.size());
  cloud->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(xyz.size());

  sensor_msgs::PointCloud2Iterator<float> ix(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(*cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> irgb(*cloud, "rgb");
  for (const auto & p : xyz) {
    *ix = p.x(); *iy = p.y(); *iz = p.z();
    irgb[0] = output_r_; irgb[1] = output_g_; irgb[2] = output_b_;
    ++ix; ++iy; ++iz; ++irgb;
  }
  cloud_pub_->publish(std::move(cloud));
}

void PerceptionNode::_publishCentroid(
  const Eigen::Vector3f & c,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  msg->header.frame_id = frame_id;
  msg->header.stamp = stamp;
  msg->point.x = c.x();
  msg->point.y = c.y();
  msg->point.z = c.z();
  centroid_pub_->publish(std::move(msg));
}

void PerceptionNode::_publishWorkspaceMarker()
{
  if (!marker_pub_) return;
  auto msg = visualization_msgs::msg::Marker();
  // 工作空间可视化固定在 target_frame。
  msg.header.frame_id = target_frame_;
  msg.header.stamp = this->now();
  msg.ns = "workspace";
  msg.id = 0;
  msg.type = visualization_msgs::msg::Marker::CUBE;
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.lifetime = rclcpp::Duration::from_seconds(0.0);

  const Eigen::Vector3f c = obb_box_.center();
  const Eigen::Vector3f h = obb_box_.halfExtents();
  const Eigen::Quaternionf q(obb_box_.rotation());
  msg.pose.position.x = c.x();
  msg.pose.position.y = c.y();
  msg.pose.position.z = c.z();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  msg.scale.x = h.x() * 2.0f;
  msg.scale.y = h.y() * 2.0f;
  msg.scale.z = h.z() * 2.0f;
  msg.color.r = marker_r_;
  msg.color.g = marker_g_;
  msg.color.b = marker_b_;
  msg.color.a = marker_a_;
  marker_pub_->publish(msg);
}

}  // namespace pc_processor
