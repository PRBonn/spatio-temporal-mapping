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

#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/CorrespondenceChecker.h>
#include <open3d/utility/Eigen.h>

#include <eigen3/Eigen/Dense>

#include "VoxelizedPointCloud.hpp"

namespace aid4crop {
namespace core {

std::tuple<Eigen::Matrix4d, float> integrate_wheel_odometry(
    const Eigen::Matrix4d&, const Eigen::Matrix4d&, const Eigen::Matrix4d&,
    const float, const float = 0.5);

std::tuple<Eigen::Matrix4d, float> constant_velocity_model(
    const Eigen::Matrix4d&, const Eigen::Matrix4d&);

std::tuple<Eigen::Matrix4d, open3d::pipelines::registration::CorrespondenceSet>
correct_pose_with_icp(const VoxelizedPointCloud&,
                      const open3d::geometry::PointCloud&,
                      const Eigen::Matrix4d&, const float = 0.05);

std::tuple<Eigen::Matrix4d, bool> global_relocalization(
    const int, const std::vector<int>&, const std::vector<Eigen::Matrix4d>&);

int match_images_by_pose(const Eigen::Matrix4d&,
                         const std::vector<Eigen::Matrix4d>&);

// Returns a vector where, in position i, we have the id of the reference img
// corresponding to the query img i
std::vector<int> load_vpr_results(const std::string&, const size_t);

}  // namespace core
}  // namespace aid4crop
