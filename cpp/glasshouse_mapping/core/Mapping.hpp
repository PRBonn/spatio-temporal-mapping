// MIT License
//
// Copyright (c) 2024 Luca Lobefaro, Meher V.R. Malladi, Tiziano Guadagnino,
// Cyrill Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <fcntl.h>
#include <open3d/camera/PinholeCameraIntrinsic.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/RGBDImage.h>

#include <csignal>
#include <cstddef>
#include <eigen3/Eigen/Dense>

#include "SensorInfo.hpp"
#include "VoxelizedPointCloud.hpp"
#include "utils/nanoflann/nanoflannWrapper.hpp"

using uint = unsigned int;
using PointCloud_ptr = std::shared_ptr<open3d::geometry::PointCloud>;
using Vector3dMatches =
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>;

namespace aid4crop {
namespace core {

// Extraction functions
VoxelizedPointCloud extract_point_cloud(std::vector<SensorInfo>&,
                                        const Eigen::Matrix4d&, const float,
                                        const float = 0.6, const float = 1.2,
                                        const int = 2, const int = 0,
                                        const int = -1);

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
unproject_matches(
    const Eigen::MatrixX2d& kps1, const Eigen::MatrixX2d& kps2,
    const std::vector<std::tuple<int, int, float>>& matches,
    const std::shared_ptr<open3d::geometry::RGBDImage>& rgbd_img1,
    const std::shared_ptr<open3d::geometry::RGBDImage>& rgbd_img2,
    const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2,
    const open3d::camera::PinholeCameraIntrinsic& camera_intrinsics1,
    const open3d::camera::PinholeCameraIntrinsic& camera_intrinsics2,
    std::vector<bool>& is_point_associated, const nanoflann_wrapper& ref_kdtree,
    const nanoflann_wrapper& query_kdtree, const double& distance_th = 0.5,
    const float& min_z_th = 0.6, const float& max_z_th = 1.2);

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
filter_matches(const std::vector<Eigen::Vector3d>& source_pts,
               const std::vector<Eigen::Vector3d>& target_pts,
               const VoxelizedPointCloud& source_pcd,
               const VoxelizedPointCloud& target_pcd);

Eigen::MatrixXd extract_given_keypoints(
    const Eigen::MatrixX2d&,
    const std::shared_ptr<open3d::geometry::RGBDImage>&, const Eigen::Matrix4d&,
    const open3d::camera::PinholeCameraIntrinsic&);

// Cleaning functions
VoxelizedPointCloud clean_point_cloud(const VoxelizedPointCloud&);

}  // namespace core
}  // namespace aid4crop
