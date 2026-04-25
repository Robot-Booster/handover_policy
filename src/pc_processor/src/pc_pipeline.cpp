#include "pc_processor/pc_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include <omp.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace pc_processor
{

// ---- CameraIntrinsics ---------------------------------------------------

CameraIntrinsics CameraIntrinsics::fromCameraInfo(const sensor_msgs::msg::CameraInfo & info)
{
  CameraIntrinsics k;
  k.fx = static_cast<float>(info.k[0]);
  k.fy = static_cast<float>(info.k[4]);
  k.cx = static_cast<float>(info.k[2]);
  k.cy = static_cast<float>(info.k[5]);
  return k;
}

// ---- ObbBox -------------------------------------------------------------

namespace
{
Eigen::Matrix3f _eulerZyxDeg(float yaw_deg, float pitch_deg, float roll_deg)
{
  const float y = yaw_deg * static_cast<float>(M_PI) / 180.0f;
  const float p = pitch_deg * static_cast<float>(M_PI) / 180.0f;
  const float r = roll_deg * static_cast<float>(M_PI) / 180.0f;
  // ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  const Eigen::AngleAxisf Rz(y, Eigen::Vector3f::UnitZ());
  const Eigen::AngleAxisf Ry(p, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf Rx(r, Eigen::Vector3f::UnitX());
  return (Rz * Ry * Rx).matrix();
}
}  // namespace

ObbBox ObbBox::loadFromJson(const std::string & path)
{
  std::ifstream fs(path);
  if (!fs.is_open()) {
    throw std::runtime_error("ObbBox: cannot open file: " + path);
  }
  nlohmann::json j;
  fs >> j;

  ObbBox box;
  const auto & c = j.at("center");
  const auto & h = j.at("half_extents");
  box.center_ = Eigen::Vector3f(c.at(0).get<float>(), c.at(1).get<float>(), c.at(2).get<float>());
  box.half_ = Eigen::Vector3f(h.at(0).get<float>(), h.at(1).get<float>(), h.at(2).get<float>());
  const float yaw = j.at("yaw_deg").get<float>();
  const float pitch = j.at("pitch_deg").get<float>();
  const float roll = j.at("roll_deg").get<float>();
  const std::string order = j.value("euler_deg_order", std::string("ZYX"));
  if (order != "ZYX") {
    throw std::runtime_error("ObbBox: only ZYX euler order supported, got: " + order);
  }
  const Eigen::Matrix3f R = _eulerZyxDeg(yaw, pitch, roll);
  box.rot_ = R;
  box.rot_t_ = R.transpose();
  box.invert_ = j.value("invert", false);
  return box;
}

// ---- RoiMask ------------------------------------------------------------

RoiMask RoiMask::loadFromFile(const std::string & path)
{
  RoiMask m;
  const cv::Mat raw = cv::imread(path, cv::IMREAD_UNCHANGED);
  if (raw.empty()) {
    throw std::runtime_error("RoiMask: imread failed: " + path);
  }
  cv::Mat gray;
  if (raw.channels() == 1) {
    gray = raw;
  } else if (raw.channels() == 3) {
    cv::cvtColor(raw, gray, cv::COLOR_BGR2GRAY);
  } else if (raw.channels() == 4) {
    cv::cvtColor(raw, gray, cv::COLOR_BGRA2GRAY);
  } else {
    throw std::runtime_error("RoiMask: unsupported channels: " + std::to_string(raw.channels()));
  }
  if (gray.empty() || !gray.isContinuous()) {
    throw std::runtime_error("RoiMask: invalid buffer");
  }
  m.width_ = static_cast<uint32_t>(gray.cols);
  m.height_ = static_cast<uint32_t>(gray.rows);
  m.data_.assign(gray.data, gray.data + gray.total());
  m.ready_ = true;
  return m;
}

// ---- fusedProject -------------------------------------------------------

void fusedProject(
  const sensor_msgs::msg::Image & depth_msg,
  const CameraIntrinsics & k,
  const RoiMask & roi,
  const sensor_msgs::msg::Image * seg_mask_msg,
  std::vector<Point> & out_points)
{
  out_points.clear();
  const uint32_t W = depth_msg.width;
  const uint32_t H = depth_msg.height;
  if (W == 0 || H == 0) return;
  const auto * depth = reinterpret_cast<const float *>(depth_msg.data.data());

  const uint8_t * roi_data = nullptr;
  if (roi.ready() && roi.width() == W && roi.height() == H) {
    roi_data = roi.data();
  }

  const uint8_t * seg_data = nullptr;
  if (seg_mask_msg && seg_mask_msg->width == W && seg_mask_msg->height == H &&
      seg_mask_msg->encoding == "mono8" && seg_mask_msg->step == W)
  {
    seg_data = seg_mask_msg->data.data();
  }

  // 输出桶先 reserve，减少 push_back 重分配
  out_points.reserve(static_cast<size_t>(W) * H / 4);

  #pragma omp parallel
  {
    std::vector<Point> tls;
    tls.reserve(static_cast<size_t>(W) * H / (omp_get_num_threads() * 2));
    #pragma omp for schedule(static) nowait
    for (int v = 0; v < static_cast<int>(H); ++v) {
      const size_t row = static_cast<size_t>(v) * W;
      for (uint32_t u = 0; u < W; ++u) {
        const size_t idx = row + u;
        if (roi_data && roi_data[idx] == 0) continue;
        const float z = depth[idx];
        if (!std::isfinite(z) || z <= 0.0f) continue;
        const float x = (static_cast<float>(u) - k.cx) * z / k.fx;
        const float y = (static_cast<float>(v) - k.cy) * z / k.fy;
        const uint8_t is_tgt = (seg_data && seg_data[idx] > 0) ? 1 : 0;
        tls.push_back(Point{x, y, z, is_tgt});
      }
    }
    #pragma omp critical
    out_points.insert(out_points.end(), tls.begin(), tls.end());
  }
}

// ---- VoxelHashGrid ------------------------------------------------------

namespace
{
// 每轴 21 位签名编码，范围 ±1,048,576 体素，5mm 格子下约 ±5km 场景足够
constexpr int32_t kVoxBits = 21;
constexpr int32_t kVoxShift = 1 << (kVoxBits - 1);
constexpr uint64_t kVoxMask = (1ULL << kVoxBits) - 1ULL;
}  // namespace

int64_t VoxelHashGrid::_packKey(int32_t ix, int32_t iy, int32_t iz)
{
  const uint64_t a = static_cast<uint64_t>(ix + kVoxShift) & kVoxMask;
  const uint64_t b = static_cast<uint64_t>(iy + kVoxShift) & kVoxMask;
  const uint64_t c = static_cast<uint64_t>(iz + kVoxShift) & kVoxMask;
  return static_cast<int64_t>(a | (b << kVoxBits) | (c << (2 * kVoxBits)));
}

void VoxelHashGrid::_unpackKey(int64_t key, int32_t & ix, int32_t & iy, int32_t & iz)
{
  const uint64_t u = static_cast<uint64_t>(key);
  ix = static_cast<int32_t>(u & kVoxMask) - kVoxShift;
  iy = static_cast<int32_t>((u >> kVoxBits) & kVoxMask) - kVoxShift;
  iz = static_cast<int32_t>((u >> (2 * kVoxBits)) & kVoxMask) - kVoxShift;
}

void VoxelHashGrid::clear()
{
  grid_.clear();
  target_sizes_.clear();
  other_sizes_.clear();
  adjacency_.clear();
  next_target_cid_ = 0;
  next_other_cid_ = 0;
}

void VoxelHashGrid::buildFromPoints(const std::vector<Point> & points, float leaf_m)
{
  clear();
  leaf_ = leaf_m;
  inv_leaf_ = 1.0f / leaf_m;
  grid_.reserve(points.size() / 4 + 16);

  // 单遍扫入哈希 Single-pass voxel accumulation.
  for (const auto & p : points) {
    const int32_t ix = static_cast<int32_t>(std::floor(p.x * inv_leaf_));
    const int32_t iy = static_cast<int32_t>(std::floor(p.y * inv_leaf_));
    const int32_t iz = static_cast<int32_t>(std::floor(p.z * inv_leaf_));
    const int64_t key = _packKey(ix, iy, iz);
    Voxel & v = grid_[key];
    v.sum_x += p.x;
    v.sum_y += p.y;
    v.sum_z += p.z;
    ++v.count;
    v.target_points += p.is_target;
  }
}

void VoxelHashGrid::rorFilter(int min_neighbors)
{
  if (min_neighbors <= 0) return;

  // 标记要删的 key，避免边遍历边删
  std::vector<int64_t> to_erase;
  to_erase.reserve(grid_.size() / 4);

  for (const auto & kv : grid_) {
    int32_t ix, iy, iz;
    _unpackKey(kv.first, ix, iy, iz);
    int nb = 0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          if (grid_.count(_packKey(ix + dx, iy + dy, iz + dz))) {
            if (++nb >= min_neighbors) goto done_count;
          }
        }
      }
    }
done_count:
    if (nb < min_neighbors) to_erase.push_back(kv.first);
  }
  for (int64_t k : to_erase) grid_.erase(k);
}

void VoxelHashGrid::_clusterSide(
  bool target_side,
  int32_t & next_cid,
  std::unordered_map<int32_t, uint32_t> & sizes,
  int cluster_min_voxels)
{
  std::vector<int64_t> stack;
  stack.reserve(256);

  for (auto & kv : grid_) {
    Voxel & v0 = kv.second;
    if (v0.cluster_id != -1) continue;
    const bool is_tgt = (v0.target_points > 0);
    if (is_tgt != target_side) continue;

    const int32_t cid = next_cid++;
    v0.cluster_id = cid;
    stack.clear();
    stack.push_back(kv.first);
    uint32_t size = 0;
    while (!stack.empty()) {
      const int64_t k = stack.back();
      stack.pop_back();
      ++size;
      int32_t ix, iy, iz;
      _unpackKey(k, ix, iy, iz);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            const int64_t nk = _packKey(ix + dx, iy + dy, iz + dz);
            auto it = grid_.find(nk);
            if (it == grid_.end()) continue;
            Voxel & nv = it->second;
            if (nv.cluster_id != -1) continue;
            const bool n_is_tgt = (nv.target_points > 0);
            if (n_is_tgt != target_side) continue;
            nv.cluster_id = cid;
            stack.push_back(nk);
          }
        }
      }
    }
    sizes.emplace(cid, size);
  }

  // 过滤过小簇：重置 cluster_id = -2 表示已过滤
  if (cluster_min_voxels > 1) {
    std::unordered_set<int32_t> drop;
    for (const auto & s : sizes) {
      if (s.second < static_cast<uint32_t>(cluster_min_voxels)) drop.insert(s.first);
    }
    for (auto it = sizes.begin(); it != sizes.end(); ) {
      if (drop.count(it->first)) it = sizes.erase(it); else ++it;
    }
    for (auto & kv : grid_) {
      Voxel & v = kv.second;
      if (v.cluster_id >= 0 && (v.target_points > 0) == target_side && drop.count(v.cluster_id)) {
        v.cluster_id = -2;
      }
    }
  }
}

void VoxelHashGrid::clusterAndAdjacency(int cluster_min_voxels)
{
  // 先 target 侧，再 other 侧，cid 空间互不冲突（分别编号）
  _clusterSide(true, next_target_cid_, target_sizes_, cluster_min_voxels);
  _clusterSide(false, next_other_cid_, other_sizes_, cluster_min_voxels);

  // 邻接记录：遍历 other 体素，查 26 邻居命中的 target 簇
  for (auto & kv : grid_) {
    const Voxel & v = kv.second;
    if (v.cluster_id < 0) continue;
    if (v.target_points > 0) continue;
    const int32_t ocid = v.cluster_id;
    int32_t ix, iy, iz;
    _unpackKey(kv.first, ix, iy, iz);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          auto it = grid_.find(_packKey(ix + dx, iy + dy, iz + dz));
          if (it == grid_.end()) continue;
          const Voxel & nv = it->second;
          if (nv.cluster_id < 0) continue;
          if (nv.target_points == 0) continue;
          adjacency_[nv.cluster_id].insert(ocid);
        }
      }
    }
  }
}

int VoxelHashGrid::selectFinalOtherCluster() const
{
  int best_other = -1;
  uint32_t best_size = 0;
  for (const auto & kv : adjacency_) {
    // 每个 target 下挑最大 other
    int local_best = -1;
    uint32_t local_size = 0;
    for (int32_t ocid : kv.second) {
      auto it = other_sizes_.find(ocid);
      if (it == other_sizes_.end()) continue;
      if (it->second > local_size) {
        local_size = it->second;
        local_best = ocid;
      }
    }
    // 全局选最大
    if (local_size > best_size) {
      best_size = local_size;
      best_other = local_best;
    }
  }
  return best_other;
}

void VoxelHashGrid::extractCluster(
  int other_cid,
  std::vector<Eigen::Vector3f> & xyz_out,
  Eigen::Vector3f & centroid_out) const
{
  xyz_out.clear();
  Eigen::Vector3f sum(0, 0, 0);
  uint32_t total_count = 0;

  for (const auto & kv : grid_) {
    const Voxel & v = kv.second;
    if (v.cluster_id != other_cid) continue;
    if (v.target_points > 0) continue;
    const float c = static_cast<float>(v.count);
    xyz_out.emplace_back(v.sum_x / c, v.sum_y / c, v.sum_z / c);
    sum += Eigen::Vector3f(v.sum_x, v.sum_y, v.sum_z);
    total_count += v.count;
  }
  centroid_out = total_count > 0
    ? Eigen::Vector3f(sum / static_cast<float>(total_count))
    : Eigen::Vector3f::Zero();
}

}  // namespace pc_processor
