#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace pc_processor
{

// ---- 基础类型 Basic types ------------------------------------------------

struct CameraIntrinsics
{
  float fx{0.0f};
  float fy{0.0f};
  float cx{0.0f};
  float cy{0.0f};

  static CameraIntrinsics fromCameraInfo(const sensor_msgs::msg::CameraInfo & info);
};

struct Point
{
  float x;
  float y;
  float z;
  uint8_t is_target;
};

// ---- 工作区 OBB ----------------------------------------------------------

class ObbBox
{
public:
  // 从 JSON 加载 Load from JSON file (pc_mask_box schema)
  static ObbBox loadFromJson(const std::string & path);
  const Eigen::Vector3f & center() const { return center_; }
  const Eigen::Vector3f & halfExtents() const { return half_; }
  const Eigen::Matrix3f & rotation() const { return rot_; }
  bool invert() const { return invert_; }

  inline bool contains(const Eigen::Vector3f & p) const
  {
    const Eigen::Vector3f local = rot_t_ * (p - center_);
    const bool inside = (std::abs(local.x()) <= half_.x())
      && (std::abs(local.y()) <= half_.y())
      && (std::abs(local.z()) <= half_.z());
    return inside != invert_;
  }

private:
  Eigen::Vector3f center_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f half_{Eigen::Vector3f::Zero()};
  Eigen::Matrix3f rot_{Eigen::Matrix3f::Identity()};
  Eigen::Matrix3f rot_t_{Eigen::Matrix3f::Identity()};
  bool invert_{false};
};

// ---- ROI 静态掩码 --------------------------------------------------------

class RoiMask
{
public:
  // 加载 PNG → mono8 Load PNG and convert to mono8.
  static RoiMask loadFromFile(const std::string & path);

  bool ready() const { return ready_; }
  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }
  const uint8_t * data() const { return ready_ ? data_.data() : nullptr; }

private:
  std::vector<uint8_t> data_;
  uint32_t width_{0};
  uint32_t height_{0};
  bool ready_{false};
};

// ---- 融合前置：深度→点+目标标记 ----------------------------------------
// 主数据流 Main data flow: roi skip → depth check → back-project → target flag
void fusedProject(
  const sensor_msgs::msg::Image & depth_msg,
  const CameraIntrinsics & k,
  const RoiMask & roi,
  const sensor_msgs::msg::Image * seg_mask_msg,
  std::vector<Point> & out_points);

// ---- 体素哈希：下采样+ROR+聚类+邻接 四合一 -----------------------------

class VoxelHashGrid
{
public:
  struct Voxel
  {
    float sum_x{0.0f};
    float sum_y{0.0f};
    float sum_z{0.0f};
    uint32_t count{0};
    uint32_t target_points{0};
    int32_t cluster_id{-1};
  };

  void clear();

  // 主数据流 Main data flow stages.
  void buildFromPoints(const std::vector<Point> & points, float leaf_m);
  void rorFilter(int min_neighbors);
  void clusterAndAdjacency(int cluster_min_voxels);

  // 选出全局最大的「附属于某目标」other 簇，返回 other_cid；无则返回 -1
  int selectFinalOtherCluster() const;

  // 导出 other 簇内体素质心 + 加权中心
  void extractCluster(
    int other_cid,
    std::vector<Eigen::Vector3f> & xyz_out,
    Eigen::Vector3f & centroid_out) const;

private:
  float leaf_{0.005f};
  float inv_leaf_{200.0f};
  std::unordered_map<int64_t, Voxel> grid_;

  std::unordered_map<int32_t, uint32_t> target_sizes_;
  std::unordered_map<int32_t, uint32_t> other_sizes_;
  std::unordered_map<int32_t, std::unordered_set<int32_t>> adjacency_;

  int32_t next_target_cid_{0};
  int32_t next_other_cid_{0};

  // 内部私有 Internal helpers.
  static int64_t _packKey(int32_t ix, int32_t iy, int32_t iz);
  static void _unpackKey(int64_t key, int32_t & ix, int32_t & iy, int32_t & iz);
  void _clusterSide(bool target_side, int32_t & next_cid, std::unordered_map<int32_t, uint32_t> & sizes, int cluster_min_voxels);
};

// ---- 聚合缓冲 Aggregated buffers ----------------------------------------

struct PipelineBuffers
{
  std::vector<Point> points;
  std::vector<Eigen::Vector3f> final_xyz;
  Eigen::Vector3f final_centroid{Eigen::Vector3f::Zero()};

  PipelineBuffers() { points.reserve(640 * 480); final_xyz.reserve(8192); }
  void clear() { points.clear(); final_xyz.clear(); }
};

}  // namespace pc_processor
