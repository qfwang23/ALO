#pragma once

#include <sophus/se3.hpp>

namespace kiss_icp {

struct AdaptiveThreshold {
    explicit AdaptiveThreshold(double initial_threshold, double min_motion_th, double max_range)
        : initial_threshold_(initial_threshold),
          min_motion_th_(min_motion_th),
          max_range_(max_range) {}

    /// Update the current belief of the deviation from the prediction model
    inline void UpdateModelDeviation(const Sophus::SE3d &current_deviation) {
        model_deviation_ = current_deviation;
    }

    /// Returns the KISS-ICP adaptive threshold used in registration
    double ComputeThreshold();

    // configurable parameters
    double initial_threshold_;
    double min_motion_th_;
    double max_range_;

    // Local cache for ccomputation
    double model_error_sse2_ = 0;
    int num_samples_ = 0;
    Sophus::SE3d model_deviation_ = Sophus::SE3d();

    std::vector<double> errors_;
};

}  // namespace kiss_icp
