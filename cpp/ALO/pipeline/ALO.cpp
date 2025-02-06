
#include "KissICP.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"

#include <iostream>
#include <fstream>  // 引入文件流头文件

namespace kiss_icp::pipeline {

KissICP::Vector3dVectorTuple KissICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame) {

        // 打开一个文件写入模式，如果文件不存在则创建它
    std::ofstream log_file("/home/wqf/4LO_code/output_log.txt", std::ios::app);  // 以追加模式打开文件

    // 获取当前时间戳
    auto current_time = std::chrono::steady_clock::now();
    
    // 计算时间间隔 delta_time，单位为秒
    std::chrono::duration<double> elapsed_seconds = current_time - last_time_;
    double delta_time = elapsed_seconds.count();

    // 更新 last_time_ 为当前时间
    last_time_ = current_time;

    // Preprocess the input cloud
    const auto &cropped_frame = Preprocess(frame, config_.max_range, config_.min_range);

    // Voxelize
    const auto &[source, frame_downsample] = Voxelize(cropped_frame);

    // Get motion prediction and adaptive_threshold
    const double sigma = GetAdaptiveThreshold();

    // Compute initial_guess for ICP
    const auto prediction = GetPredictionModel(sigma);
    const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess = last_pose * prediction;

    // Run ICP
    const Sophus::SE3d new_pose = kiss_icp::RegisterFrame(source,         //
                                                          local_map_,     //
                                                          initial_guess,  //
                                                          3.0 * sigma,    //
                                                          sigma / 3.0);
 // 计算速度 (translation velocity) 和加速度 (translation acceleration)
    Eigen::Vector3d velocity = (new_pose.translation() - last_pose.translation()) / delta_time;
    Eigen::Vector3d acceleration = (velocity - last_velocity_) / delta_time;

    // 设置速度和加速度的最大阈值，防止权重过大
    const double max_velocity_threshold = 10.0;   // 速度的最大阈值
    const double max_acceleration_threshold = 5.0;  // 加速度的最大阈值



    // 归一化速度和加速度，确保它们在 [0, 1] 范围内
    double normalized_velocity = std::min(velocity.norm() / max_velocity_threshold, 1.0);
    double normalized_acceleration = std::min(acceleration.norm() / max_acceleration_threshold, 1.0);

    // 结合速度和加速度计算动态权重，并限制在 [0, 1.5] 范围内
    double dynamic_weight = std::min(normalized_velocity + normalized_acceleration, 1.5);

    // 计算模型偏差，分别应用动态权重
    const auto model_deviation = initial_guess.inverse() * new_pose;

    // 对平移部分应用动态权重
    Eigen::Vector3d weighted_translation = dynamic_weight * model_deviation.translation();

    // 对旋转部分保持原有的旋转不加权（如果需要，也可以基于速度/加速度对旋转进行调整）
    Eigen::Quaterniond weighted_rotation = model_deviation.unit_quaternion();

    // 构造新的带权重的模型偏差
    Sophus::SE3d weighted_model_deviation(weighted_rotation, weighted_translation);
    adaptive_threshold_.UpdateModelDeviation(weighted_model_deviation);

    // 保存最新的速度
    last_velocity_ = velocity;

    // 更新 local map 和位姿
    local_map_.Update(frame_downsample, new_pose);
    poses_.push_back(new_pose);
    return {frame, source};
}

KissICP::Vector3dVectorTuple KissICP::Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
    const auto voxel_size = config_.voxel_size;
    // const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size*2 );

    // const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 2.3);

    return {source, frame_downsample};
}

double KissICP::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_threshold;
    }
    return adaptive_threshold_.ComputeThreshold();
}

Sophus::SE3d KissICP::GetPredictionModel(double sigma) const {
  const size_t N = poses_.size();

    // 如果位姿少于2个，返回恒等变换
    if (N < 2) return Sophus::SE3d();  
    
    // 如果只有2个位姿，直接返回相对变换
    if (N < 3) {
        return poses_[N - 2].inverse() * poses_[N - 1];
    }

    // 使用最近的3个位姿进行平移和旋转的加权估计
    Eigen::Vector3d cumulative_translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond cumulative_rotation(1, 0, 0, 0);  // 单位四元数

    // 根据sigma动态调整权重，sigma越大越依赖最近的位姿
    double recent_weight = std::min(1.0, sigma / 10.0);  // 平滑调整权重，避免过大变化
    std::array<double, 2> weights = {1.0 - recent_weight, recent_weight};  // 基于sigma动态调整权重
    double weight_sum = weights[0] + weights[1];

    for (size_t i = N - 3; i < N - 1; ++i) {
        Sophus::SE3d delta_pose = poses_[i].inverse() * poses_[i + 1];
        cumulative_translation += weights[i - (N - 3)] * delta_pose.translation();
        cumulative_rotation = cumulative_rotation.slerp(weights[i - (N - 3)], delta_pose.unit_quaternion());
    }

    Eigen::Vector3d average_translation = cumulative_translation / weight_sum;  // 加权平移
    Eigen::Quaterniond average_rotation = cumulative_rotation.normalized();  // 正则化四元数

    // 返回加权的位姿估计
    return Sophus::SE3d(average_rotation, average_translation);
}


bool KissICP::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}

}  // namespace kiss_icp::pipeline
