#include "Threshold.hpp"

#include <Eigen/Core>
#include <cmath>

namespace {
//myadd
double ComputeWeight(double current_error, double previous_error) {
    if (previous_error == 0.0) {
        return 1.0;  
    }

    double error_change_rate = (current_error - previous_error) / (previous_error + 1e-6);
    double sigmoid = 1.0 / (1.0 + exp(-error_change_rate));  

    double adjusted_change = 2.0 * (sigmoid - 0.5);  
    double weight = 1.0 + 0.25 * adjusted_change;  

    return weight;
}

    double ComputeModelError(const Sophus::SE3d &model_deviation, double max_range) {
        const double theta = Eigen::AngleAxisd(model_deviation.rotationMatrix()).angle();
        const double delta_rot = max_range * tanh(0.1 * theta);  
        const double delta_trans = model_deviation.translation().norm();
        return delta_trans + delta_rot;

    }
}


namespace kiss_icp {

double AdaptiveThreshold::ComputeThreshold() {
double model_error = ComputeModelError(model_deviation_, max_range_);

double weight = 1.0;
     
 if (!errors_.empty()) {
        weight = ComputeWeight(model_error, errors_.back());
}
errors_.push_back(model_error);
     
    if (model_error > min_motion_th_) {

            model_error_sse2_ += weight * model_error * model_error;
            num_samples_++;
    }

    if (num_samples_ < 1) {
        return initial_threshold_;
    }
    return std::sqrt(model_error_sse2_ / num_samples_);
}

}  // namespace kiss_icp