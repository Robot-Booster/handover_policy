#pragma once

#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pc_processor/pc_pipeline.hpp"

namespace pc_processor
{

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode();

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void onSynced(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & mask_msg);

  void processFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & mask_msg);

  // 内部私有 Internal helpers.
  void _declareAndLoadParams();
  void _loadStaticResources();
  bool _transformAndCropPointsToTarget(
    std::vector<Point> & points,
    const std::string & source_frame,
    const rclcpp::Time & stamp);
  void _publishCloud(
    const std::vector<Eigen::Vector3f> & xyz,
    const std::string & frame_id,
    const rclcpp::Time & stamp);
  void _publishCentroid(
    const Eigen::Vector3f & c,
    const std::string & frame_id,
    const rclcpp::Time & stamp);
  void _publishWorkspaceMarker();
  void _logParameters() const;

  // 参数 Parameters.
  std::string depth_topic_;
  std::string depth_camera_info_topic_;
  std::string seg_mask_topic_;
  std::string cloud_topic_;
  std::string centroid_topic_;
  std::string workspace_marker_topic_;
  std::string target_frame_;
  std::string roi_mask_image_path_;
  std::string workspace_box_json_path_;
  double depth_mask_max_stamp_diff_sec_{0.05};
  bool allow_future_mask_{false};
  float voxel_leaf_size_m_{0.005f};
  int ror_min_neighbors_{5};
  int cluster_min_voxels_{20};
  uint8_t output_r_{255};
  uint8_t output_g_{0};
  uint8_t output_b_{0};
  float marker_r_{0.0f};
  float marker_g_{1.0f};
  float marker_b_{1.0f};
  float marker_a_{0.25f};

  // 静态资源 Static resources.
  ObbBox obb_box_;
  RoiMask roi_mask_;

  // 运行时缓存 Runtime caches.
  CameraIntrinsics intrinsics_;
  bool intrinsics_ready_{false};
  PipelineBuffers buffers_;
  VoxelHashGrid grid_;

  // ROS I/O.
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> mask_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace pc_processor
