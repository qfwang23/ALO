#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

namespace kiss_icp {

/// Crop the frame with max/min ranges
std::vector<Eigen::Vector3d> Preprocess(const std::vector<Eigen::Vector3d> &frame,
                                        double max_range,
                                        double min_range);

/// This function only applies for the KITTI dataset, and should NOT be used by any other dataset,
/// the original idea and part of the implementation is taking from CT-ICP(Although IMLS-SLAM
/// Originally introduced the calibration factor)
std::vector<Eigen::Vector3d> CorrectKITTIScan(const std::vector<Eigen::Vector3d> &frame);

/// Voxelize point cloud keeping the original coordinates
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
                                             double voxel_size);
}  // namespace kiss_icp
