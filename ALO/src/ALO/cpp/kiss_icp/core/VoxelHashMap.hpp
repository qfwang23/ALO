#pragma once

#include <tsl/robin_map.h>


#include <deque>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

namespace kiss_icp {
struct VoxelHashMap {
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;
    
using Vector3dVectorTupleWithNormalsAndTargets = std::tuple<Vector3dVector, Vector3dVector, Vector3dVector>;

    using Voxel = Eigen::Vector3i;
    // struct VoxelBlock {
    //     // buffer of points with a max limit of n_points3D 体素网格中存储点云数据。每个 VoxelBlock 表示一个体素（voxel）内的点集合，并且有一个最大点数的限制。
    //     std::vector<Eigen::Vector3d> points;
    //     int num_points_;
    //     inline void AddPoint(const Eigen::Vector3d &point) {
    //         if (points.size() < static_cast<size_t>(num_points_)) points.push_back(point);
    //     }
    // };
    struct VoxelBlock {
    // buffer of points with a max limit of n_points
    std::vector<Eigen::Vector3d> points;  // 存储体素内的点
    int num_points_;  // 允许的最大点数

    // 添加新点到体素的函数，使用先进先出策略
    inline void AddPoint(const Eigen::Vector3d &point) {
        if (points.size() >= static_cast<size_t>(num_points_)) {
            // 当点数达到限制，移除第一个点 (FIFO 策略)
            points.erase(points.begin());
        }
        // 添加新点
        points.push_back(point);
    }
};

    struct VoxelHash {
        size_t operator()(const Voxel &voxel) const {
            const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
            return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
        }
    };

    explicit VoxelHashMap(double voxel_size, double max_distance, int max_points_per_voxel)
        : voxel_size_(voxel_size),
          max_distance_(max_distance),
          max_points_per_voxel_(max_points_per_voxel) {}

    Vector3dVectorTuple GetCorrespondences(const Vector3dVector &points,
                                           double max_correspondance_distance) const;

    VoxelHashMap::Vector3dVectorTupleWithNormalsAndTargets GetCorrespondencesWithNormals(
        const Vector3dVector &points, double max_correspondance_distance) const;

    inline void Clear() { map_.clear(); }
    inline bool Empty() const { return map_.empty(); }
    void Update(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &origin);
    void Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose);
    void AddPoints(const std::vector<Eigen::Vector3d> &points);


    void RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    std::vector<Eigen::Vector3d> Pointcloud() const;
   
    double voxel_size_;
    double max_distance_;
    int max_points_per_voxel_;
    tsl::robin_map<Voxel, VoxelBlock, VoxelHash> map_;


    void RemoveOldestFrame();  // 删除最旧的帧

    // 新的更新函数，使用滑动窗口策略
    void UpdateWithSlidingWindow(const Vector3dVector &points);

        // 队列用于保存最近的几帧点云数据
    std::deque<std::vector<Eigen::Vector3d>> pointcloud_history_;  // 保存滑动窗口内的点云
    size_t max_frames_= 1000;  // 最大帧数（滑动窗口大小）

};
}  // namespace kiss_icp
