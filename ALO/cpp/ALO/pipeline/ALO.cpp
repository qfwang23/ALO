
#include "ALO.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "ALO/core/Preprocessing.hpp"
#include "ALO/core/Registration.hpp"
#include "ALO/core/VoxelHashMap.hpp"

#include <iostream>
#include <fstream>  

namespace kiss_icp::pipeline {

KissICP::Vector3dVectorTuple KissICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame) {

    std::ofstream log_file("/home/wqf/4LO_code/output_log.txt", std::ios::app);


    auto current_time = std::chrono::steady_clock::now();
    
   
    std::chrono::duration<double> elapsed_seconds = current_time - last_time_;
    double delta_time = elapsed_seconds.count();

    last_time_ = current_time;


    const auto &cropped_frame = Preprocess(frame, config_.max_range, config_.min_range);


    const auto &[source, frame_downsample] = Voxelize(cropped_frame);

    const double sigma = GetAdaptiveThreshold();


    const auto prediction = GetPredictionModel(sigma);
    const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess = last_pose * prediction;

    const Sophus::SE3d new_pose = kiss_icp::RegisterFrame(source,         //
                                                          local_map_,     //
                                                          initial_guess,  //
                                                          3.0 * sigma,    //
                                                          sigma / 3.0);

    Eigen::Vector3d velocity = (new_pose.translation() - last_pose.translation()) / delta_time;
    Eigen::Vector3d acceleration = (velocity - last_velocity_) / delta_time;


    const double max_velocity_threshold = 10.0;  
    const double max_acceleration_threshold = 5.0; 


    double normalized_velocity = std::min(velocity.norm() / max_velocity_threshold, 1.0);
    double normalized_acceleration = std::min(acceleration.norm() / max_acceleration_threshold, 1.0);


    double dynamic_weight = std::min(normalized_velocity + normalized_acceleration, 1.5);


    const auto model_deviation = initial_guess.inverse() * new_pose;


    Eigen::Vector3d weighted_translation = dynamic_weight * model_deviation.translation();

    Eigen::Quaterniond weighted_rotation = model_deviation.unit_quaternion();


    Sophus::SE3d weighted_model_deviation(weighted_rotation, weighted_translation);
    adaptive_threshold_.UpdateModelDeviation(weighted_model_deviation);


    last_velocity_ = velocity;

    local_map_.Update(frame_downsample, new_pose);
    poses_.push_back(new_pose);
    return {frame, source};
}

KissICP::Vector3dVectorTuple KissICP::Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
    const auto voxel_size = config_.voxel_size;
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size*2 );

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

    if (N < 2) return Sophus::SE3d();  
    
    if (N < 3) {
        return poses_[N - 2].inverse() * poses_[N - 1];
    }

    Eigen::Vector3d cumulative_translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond cumulative_rotation(1, 0, 0, 0);  

    double recent_weight = std::min(1.0, sigma / 10.0); 
    std::array<double, 2> weights = {1.0 - recent_weight, recent_weight};  
    double weight_sum = weights[0] + weights[1];

    for (size_t i = N - 3; i < N - 1; ++i) {
        Sophus::SE3d delta_pose = poses_[i].inverse() * poses_[i + 1];
        cumulative_translation += weights[i - (N - 3)] * delta_pose.translation();
        cumulative_rotation = cumulative_rotation.slerp(weights[i - (N - 3)], delta_pose.unit_quaternion());
    }

    Eigen::Vector3d average_translation = cumulative_translation / weight_sum;  
    Eigen::Quaterniond average_rotation = cumulative_rotation.normalized();  

    return Sophus::SE3d(average_rotation, average_translation);
}


bool KissICP::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}

}  // namespace kiss_icp::pipeline
