#include "Threshold.hpp"

#include <Eigen/Core>
#include <cmath>

namespace {
//myadd
double ComputeWeight(double current_error, double previous_error) {
    if (previous_error == 0.0) {
        return 1.0;  // 使用默认权重，避免除零错误
    }

    double error_change_rate = (current_error - previous_error) / (previous_error + 1e-6);
    double sigmoid = 1.0 / (1.0 + exp(-error_change_rate));  // 使用 Sigmoid 函数进行平滑处理

    double adjusted_change = 2.0 * (sigmoid - 0.5);  // 将 Sigmoid 输出转换为 -1 到 1 范围的值
    double weight = 1.0 + 0.25 * adjusted_change;  // 调整因子，根据实际需求调整

    return weight;
}

    double ComputeModelError(const Sophus::SE3d &model_deviation, double max_range) {
        const double theta = Eigen::AngleAxisd(model_deviation.rotationMatrix()).angle();
        const double delta_rot = max_range * tanh(0.1 * theta);  // 使用双曲正切函数对大角度进行平滑处理
        const double delta_trans = model_deviation.translation().norm();
        return delta_trans + delta_rot;

    }
}


namespace kiss_icp {

double AdaptiveThreshold::ComputeThreshold() {
double model_error = ComputeModelError(model_deviation_, max_range_);

    //myadd
double weight = 1.0;
     
 if (!errors_.empty()) {
        weight = ComputeWeight(model_error, errors_.back());
}
errors_.push_back(model_error);
     
    if (model_error > min_motion_th_) {//如果计算的模型误差大于最小运动阈值 min_motion_th_，则将该误差的平方加到 model_error_sse2_ 上，并增加样本数 num_samples_。
        // model_error_sse2_ += model_error * model_error;
        // num_samples_++;

        //myadd
            model_error_sse2_ += weight * model_error * model_error;
            num_samples_++;
    }

    if (num_samples_ < 1) {
        return initial_threshold_;
    }
    return std::sqrt(model_error_sse2_ / num_samples_);
}

}  // namespace kiss_icp