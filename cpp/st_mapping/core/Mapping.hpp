// MIT License
//
// Copyright (c) 2024 Luca Lobefaro, Meher V.R. Malladi, Tiziano Guadagnino, Cyrill Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace st_mapping {

using PointsAndColor = std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>;
using PointWithColor = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using PointCloud = std::vector<PointWithColor>;
using Matches3D = std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>;

PointCloud ExtractPointCloud(const cv::Mat &rgb_img,
                             const cv::Mat &depth_img,
                             const Eigen::Matrix3d &camera_intrinsics,
                             const Sophus::SE3d &camera_extrinsics,
                             const double min_th,
                             const double max_th,
                             const int stride = 1);

PointCloud VoxelDownSample(const PointCloud &pcd, const double voxel_size);

PointCloud Threshold(const PointCloud &pcd,
                     const int axis,
                     const double min_th,
                     const double max_th);

std::vector<Eigen::Vector3d> Unproject2DPoints(const std::vector<Eigen::Vector2i> &points2D,
                                               const cv::Mat &depth_img,
                                               const Eigen::Matrix3d &camera_intrinsics,
                                               const Sophus::SE3d &camera_extrinsics,
                                               const Sophus::SE3d &pose,
                                               const double min_th,
                                               const double max_th);

Matches3D FilterMatches(const std::vector<Eigen::Vector3d> &target_points,
                        const std::vector<Eigen::Vector3d> &ref_points,
                        const double &distance_th);

}  // namespace st_mapping
