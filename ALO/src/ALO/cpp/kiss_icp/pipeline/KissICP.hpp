#pragma once

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "kiss_icp/core/Threshold.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"

#include <chrono>  // 用于时间戳

namespace kiss_icp::pipeline {

struct KISSConfig {
    // map params
    double voxel_size = 1.0;
    double max_range = 100.0;
    double min_range = 5.0;
    // int max_points_per_voxel = 20;

    int max_points_per_voxel = 40;

    

    // th parms
    double min_motion_th = 0.1;
    double initial_threshold = 2.0;

    // Motion compensation
    bool deskew = false;
};

class KissICP {
public:
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;

public:
    explicit KissICP(const KISSConfig &config)
        : config_(config),
          local_map_(config.voxel_size, config.max_range, config.max_points_per_voxel),
          adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range),
          
          last_time_(std::chrono::steady_clock::now()),  // 初始化为当前时间
          last_velocity_(Eigen::Vector3d::Zero()) {}     // 初始化速度为 0 {}

    KissICP() : KissICP(KISSConfig{}) {}

public:
    Vector3dVectorTuple RegisterFrame(const std::vector<Eigen::Vector3d> &frame);
    Vector3dVectorTuple Voxelize(const std::vector<Eigen::Vector3d> &frame) const;
    double GetAdaptiveThreshold();
    Sophus::SE3d GetPredictionModel(double sigma) const;
    bool HasMoved();

public:
    // Extra C++ API to facilitate ROS debugging
    std::vector<Eigen::Vector3d> LocalMap() const { return local_map_.Pointcloud(); };
    std::vector<Sophus::SE3d> poses() const { return poses_; };

private:
    // KISS-ICP pipeline modules
    std::vector<Sophus::SE3d> poses_;
    KISSConfig config_;
    VoxelHashMap local_map_;
    AdaptiveThreshold adaptive_threshold_;

       // 新增成员变量，用于记录时间和速度
    std::chrono::steady_clock::time_point last_time_;   // 记录上一帧的时间戳
    Eigen::Vector3d last_velocity_;                    // 记录上一帧的速度
};

}  // namespace kiss_icp::pipeline
