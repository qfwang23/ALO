#include "VoxelHashMap.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <deque>
#include <iostream>
#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace {
struct ResultTuple {
    ResultTuple(std::size_t n) {
        source.reserve(n);
        target.reserve(n);
    }
    std::vector<Eigen::Vector3d> source;
    std::vector<Eigen::Vector3d> target;
};

struct ResultTupleWithNormals {
    ResultTupleWithNormals(std::size_t n) {
        points.reserve(n);
        targets.reserve(n);
        normals.reserve(n);
    }
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> targets;
    std::vector<Eigen::Vector3d> normals;
};

}  // namespace

namespace kiss_icp {

Eigen::Vector3d FitPlaneAndGetNormal(const std::vector<Eigen::Vector3d>& neighbors) {
    if (neighbors.size() < 3) { 
        return Eigen::Vector3d::Zero();  
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& neighbor : neighbors) {
        centroid += neighbor;
    }
    centroid /= static_cast<double>(neighbors.size());

    Eigen::MatrixXd A(neighbors.size(), 3);
    for (std::size_t i = 0; i < neighbors.size(); ++i) {
        A.row(i) = neighbors[i] - centroid;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d normal = svd.matrixV().col(2);

    if (svd.singularValues()[2] < 1e-6) {
        return Eigen::Vector3d::Zero();
    }

    return normal.normalized();
}

VoxelHashMap::Vector3dVectorTuple VoxelHashMap::GetCorrespondences(
    const Vector3dVector &points, double max_correspondance_distance) const {

    auto GetClosestNeighboor = [&](const Eigen::Vector3d &point) {
        auto kx = static_cast<int>(point[0] / voxel_size_);
        auto ky = static_cast<int>(point[1] / voxel_size_);
        auto kz = static_cast<int>(point[2] / voxel_size_);
        std::vector<Voxel> voxels;

        int range = 1; 
        voxels.reserve((2 * range + 1) * (2 * range + 1) * (2 * range + 1)); 

        for (int i = kx - range; i < kx + range + 1; ++i) {
            for (int j = ky - range; j < ky + range + 1; ++j) {
                for (int k = kz - range; k < kz + range + 1; ++k) {
                    voxels.emplace_back(i, j, k);
                }
            }
        }

        using Vector3dVector = std::vector<Eigen::Vector3d>;
        Vector3dVector neighbors;
        neighbors.reserve((2 * range + 1) * (2 * range + 1) * (2 * range + 1) * max_points_per_voxel_);

        for (const auto &voxel : voxels) {
            auto search = map_.find(voxel);
            if (search != map_.end()) {
                const auto &points = search->second.points;
                neighbors.insert(neighbors.end(), points.begin(), points.end());  // 插入点
            }
        }

        Eigen::Vector3d closest_neighbor;
        double closest_distance2 = std::numeric_limits<double>::max();
        for (const auto &neighbor : neighbors) {
            double distance2 = (neighbor - point).squaredNorm();
            if (distance2 < closest_distance2) {
                closest_neighbor = neighbor;
                closest_distance2 = distance2;
            }
        }

        return closest_neighbor;
    };

    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    const auto [source, target] = tbb::parallel_reduce(
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        ResultTuple(points.size()),
        [max_correspondance_distance, &GetClosestNeighboor](
            const tbb::blocked_range<points_iterator> &r, ResultTuple res) -> ResultTuple {
            auto &[src, tgt] = res;
            src.reserve(r.size());
            tgt.reserve(r.size());
            for (const auto &point : r) {
                Eigen::Vector3d closest_neighboors = GetClosestNeighboor(point);
                if ((closest_neighboors - point).norm() < max_correspondance_distance) {
                    src.emplace_back(point);
                    tgt.emplace_back(closest_neighboors);
                }
            }
            return res;
        },
        [](ResultTuple a, const ResultTuple &b) -> ResultTuple {
            auto &[src, tgt] = a;
            const auto &[srcp, tgtp] = b;
            src.insert(src.end(),
                       std::make_move_iterator(srcp.begin()), std::make_move_iterator(srcp.end()));
            tgt.insert(tgt.end(),
                       std::make_move_iterator(tgtp.begin()), std::make_move_iterator(tgtp.end()));
            return a;
        });
    return std::make_tuple(source, target);
}

VoxelHashMap::Vector3dVectorTupleWithNormalsAndTargets VoxelHashMap::GetCorrespondencesWithNormals(
    const Vector3dVector &points, double max_correspondance_distance) const {

    auto GetKClosestNeighbors = [&](const Eigen::Vector3d &point) {
        auto kx = static_cast<int>(point[0] / voxel_size_);
        auto ky = static_cast<int>(point[1] / voxel_size_);
        auto kz = static_cast<int>(point[2] / voxel_size_);
        std::vector<Voxel> voxels;

        int range = 1;
        voxels.reserve((2 * range + 1) * (2 * range + 1) * (2 * range + 1));

        for (int i = kx - range; i <= kx + range; ++i) {
            for (int j = ky - range; j <= ky + range; ++j) {
                for (int k = kz - range; k <= kz + range; ++k) {
                    voxels.emplace_back(i, j, k);
                }
            }
        }
        std::vector<Eigen::Vector3d> neighbors;
        neighbors.reserve(4 * max_points_per_voxel_);

        for (const auto &voxel : voxels) {
            auto search = map_.find(voxel);
            if (search != map_.end()) {
                const auto &points = search->second.points;
                neighbors.insert(neighbors.end(), points.begin(), points.end());
            }
        }

        std::partial_sort(neighbors.begin(), neighbors.begin() + std::min(4UL, neighbors.size()), neighbors.end(),
                          [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
                              return (a - point).squaredNorm() < (b - point).squaredNorm();
                          });
        if (neighbors.size() > 4) {
            neighbors.resize(4);  
        }

        return neighbors;
    };

    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    const auto [src_points, tgt_points, normals] = tbb::parallel_reduce(
        tbb::blocked_range<points_iterator>(points.cbegin(), points.cend(), 100),  
        ResultTupleWithNormals(points.size()),
        [max_correspondance_distance, &GetKClosestNeighbors](
            const tbb::blocked_range<points_iterator> &r, ResultTupleWithNormals res) -> ResultTupleWithNormals {
            auto &[src, tgt, normal_vectors] = res;
            src.reserve(r.size());
            tgt.reserve(r.size());
            normal_vectors.reserve(r.size());
            for (const auto &point : r) {
                auto neighbors = GetKClosestNeighbors(point);
                if (neighbors.size() < 3) continue;  
                Eigen::Vector3d normal = FitPlaneAndGetNormal(neighbors);
                if (normal.isZero()) continue;  

                double max_distance_to_plane = 0.0;
                for (const auto &neighbor : neighbors) {
                    double distance_to_plane = std::fabs((neighbor - point).dot(normal));
                    max_distance_to_plane = std::max(max_distance_to_plane, distance_to_plane);
                }

                const double distance_threshold = max_correspondance_distance;  
                if (max_distance_to_plane > distance_threshold) continue;

                Eigen::Vector3d closest_neighbor;
                double closest_distance2 = std::numeric_limits<double>::max();
                for (const auto &neighbor : neighbors) {
                    double distance2 = (neighbor - point).squaredNorm();
                    if (distance2 < closest_distance2) {
                        closest_neighbor = neighbor;
                        closest_distance2 = distance2;
                    }
                }
                src.emplace_back(point);
                tgt.emplace_back(closest_neighbor);
                normal_vectors.emplace_back(normal);
            }
            return res;
        },
        [](ResultTupleWithNormals a, const ResultTupleWithNormals &b) -> ResultTupleWithNormals {
            auto &[src, tgt, normal_vectors] = a;
            const auto &[src_b, tgt_b, normal_vectors_b] = b;
            src.insert(src.end(), std::make_move_iterator(src_b.begin()), std::make_move_iterator(src_b.end()));
            tgt.insert(tgt.end(), std::make_move_iterator(tgt_b.begin()), std::make_move_iterator(tgt_b.end()));
            normal_vectors.insert(normal_vectors.end(),
                                  std::make_move_iterator(normal_vectors_b.begin()),
                                  std::make_move_iterator(normal_vectors_b.end()));
            return a;
        });

    return std::make_tuple(src_points, tgt_points, normals);
}

std::vector<Eigen::Vector3d> VoxelHashMap::Pointcloud() const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(max_points_per_voxel_ * map_.size());
    for (const auto &[voxel, voxel_block] : map_) {
        (void)voxel;
        for (const auto &point : voxel_block.points) {
            points.push_back(point);
        }
    }
    return points;
}

void VoxelHashMap::Update(const Vector3dVector &points, const Eigen::Vector3d &origin) {
    AddPoints(points);  
    RemovePointsFarFromLocation(origin); 

}

void VoxelHashMap::Update(const Vector3dVector &points, const Sophus::SE3d &pose) {
    Vector3dVector points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    const Eigen::Vector3d &origin = pose.translation();
    Update(points_transformed, origin);
}

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points) {
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        auto voxel = Voxel((point / voxel_size_).template cast<int>());
        auto search = map_.find(voxel);

        if (search != map_.end()) {

            auto &voxel_block = map_.at(voxel); 

            if (voxel_block.points.size() >= static_cast<size_t>(max_points_per_voxel_)) {
                voxel_block.points.erase(voxel_block.points.begin());
            }

            voxel_block.AddPoint(point);

        } else {

            map_.insert({voxel, VoxelBlock{{point}, max_points_per_voxel_}});
        }
    });
}

void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    for (const auto &[voxel, voxel_block] : map_) {
        const auto &pt = voxel_block.points.front();
        const auto max_distance2 = 9*max_distance_ * max_distance_;
        if ((pt - origin).squaredNorm() > (max_distance2)) {
            map_.erase(voxel);
        }
    }
}

void VoxelHashMap::UpdateWithSlidingWindow(const Vector3dVector &points) {

    pointcloud_history_.push_back(points);

    if (pointcloud_history_.size() > max_frames_) {
        RemoveOldestFrame();
    }

    AddPoints(points);
}


void VoxelHashMap::RemoveOldestFrame() {
    const auto &oldest_points = pointcloud_history_.front();

    for (const auto &point : oldest_points) {
        auto voxel = Voxel((point / voxel_size_).template cast<int>());  
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto &voxel_block = search->second;
            auto &modifiable_points = const_cast<std::vector<Eigen::Vector3d>&>(voxel_block.points);

            auto it = std::find(modifiable_points.begin(), modifiable_points.end(), point);
            if (it != modifiable_points.end()) {
                modifiable_points.erase(it); 
            }
            if (modifiable_points.empty()) {
                map_.erase(voxel);
            }
        }
    }

    pointcloud_history_.pop_front();
}

}
