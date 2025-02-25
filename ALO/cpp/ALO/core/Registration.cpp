#include "Registration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>  // 用于设置线程数

#include <algorithm>
#include <cmath>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace {

inline double square(double x) { return x * x; }

struct ResultTuple {
    ResultTuple() {
        JTJ.setZero();
        JTr.setZero();
    }

    ResultTuple operator+(const ResultTuple &other) {
        this->JTJ += other.JTJ;
        this->JTr += other.JTr;
        return *this;
    }

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
};

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

constexpr int MAX_NUM_ITERATIONS_ = 300; 
constexpr double ESTIMATION_THRESHOLD_ = 0.00005;

std::tuple<Eigen::Matrix6d, Eigen::Vector6d> BuildLinearSystem(
    const std::vector<Eigen::Vector3d> &source,
    const std::vector<Eigen::Vector3d> &target,
    const std::vector<Eigen::Vector3d> &normals,
    double kernel) {
    
    ResultTuple result = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, source.size()),
        ResultTuple(),
        [&](const tbb::blocked_range<size_t> &r, ResultTuple res) -> ResultTuple {
            auto Weight = [&](double residual2) {
                return square(kernel) / square(kernel + residual2);  
            };
            auto &[JTJ_private, JTr_private] = res;
            for (size_t i = r.begin(); i != r.end(); ++i) {
                const Eigen::Vector3d residual = (source[i] - target[i]).dot(normals[i]) * normals[i];  
                Eigen::Matrix3_6d J_r;
                J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source[i]);

                const double w = Weight(residual.squaredNorm());  
                JTJ_private.noalias() += J_r.transpose() * w * J_r; 
                JTr_private.noalias() += J_r.transpose() * w * residual; 
            }
            return res;
        },
        [](ResultTuple a, const ResultTuple &b) -> ResultTuple {
            return a + b;
        }
    );

    return std::make_tuple(result.JTJ, result.JTr);
}

}  // namespace

namespace kiss_icp {

Sophus::SE3d RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                           const VoxelHashMap &voxel_map,
                           const Sophus::SE3d &initial_guess,
                           double max_correspondence_distance,
                           double kernel) {
    if (voxel_map.Empty()) return initial_guess;

    const int max_threads = std::thread::hardware_concurrency();  
    tbb::task_arena arena(max_threads); 

    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess, source);

    // ICP循环
    Sophus::SE3d T_icp = Sophus::SE3d();
    arena.execute([&] {
        for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
            const auto &[src, tgt, normals] = voxel_map.GetCorrespondencesWithNormals(source, max_correspondence_distance);

            if (src.size() < 3) break;

            const auto &[JTJ, JTr] = BuildLinearSystem(src, tgt, normals, kernel);

            Eigen::LLT<Eigen::Matrix6d> llt(JTJ);
            if (llt.info() == Eigen::NumericalIssue) {
                break;  
            }

            const Eigen::Vector6d dx = llt.solve(-JTr);
            const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);

            TransformPoints(estimation, source);

            T_icp = estimation * T_icp;

            if (dx.norm() < ESTIMATION_THRESHOLD_) break;
        }
    });

    return T_icp * initial_guess;
}

}  // namespace kiss_icp
