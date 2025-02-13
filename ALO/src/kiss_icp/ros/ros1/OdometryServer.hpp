#pragma once

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace kiss_icp_ros {

class OdometryServer {
public:
    /// OdometryServer constructor
    OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

private:
    /// Register new frame
    void RegisterFrame(const sensor_msgs::PointCloud2::ConstPtr &msg);

    // 添加保存轨迹
    void SavePoseToKITTIFormat(const Sophus::SE3d &pose, const std::string &filename) const;
    void SavePoseToTUMFormat(const Sophus::SE3d &pose,
                             const ros::Time &timestamp,
                             const std::string &filename) const;


    /// Stream the estimated pose to ROS
    void PublishOdometry(const Sophus::SE3d &pose,
                         const ros::Time &stamp,
                         const std::string &cloud_frame_id);

    /// Stream the debugging point clouds for visualization (if required)
    void PublishClouds(const std::vector<Eigen::Vector3d> frame,
                       const std::vector<Eigen::Vector3d> keypoints,
                       const ros::Time &stamp,
                       const std::string &cloud_frame_id);

    /// Utility function to compute transformation using tf tree
    Sophus::SE3d LookupTransform(const std::string &target_frame,
                                 const std::string &source_frame) const;

    /// Ros node stuff
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    // int queue_size_{1};


    int queue_size_{5};


    /// Tools for broadcasting TFs.
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    bool publish_odom_tf_;
    bool publish_debug_clouds_;

    /// Data subscribers.
    ros::Subscriber pointcloud_sub_;

    /// Data publishers.
    ros::Publisher odom_publisher_;
    ros::Publisher frame_publisher_;
    ros::Publisher kpoints_publisher_;
    ros::Publisher map_publisher_;
    ros::Publisher traj_publisher_;
    nav_msgs::Path path_msg_;

    /// KISS-ICP
    kiss_icp::pipeline::KissICP odometry_;
    kiss_icp::pipeline::KISSConfig config_;

    /// Global/map coordinate frame.
    std::string odom_frame_{"odom"};
    std::string base_frame_{};
};

}  // namespace kiss_icp_ros
