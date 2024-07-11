// ----------------------------------------------------------------------------
// NOTE: This file has been adapted from the Kiss-ICP project, but copyright
// still belongs to Kiss-ICP. All rights reserved
// ----------------------------------------------------------------------------
// -              Kiss-ICP: https://github.com/PRBonn/kiss-icp                -
// ----------------------------------------------------------------------------
// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
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

#include <eigen3/Eigen/Dense>
#include <vector>

namespace voxel_hash_map {

using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using pair_distance_t = std::tuple<double, Eigen::Vector3d>;
using Vector3dVector = std::vector<Eigen::Vector3d>;

struct Voxel {
  Voxel(int x, int y, int z) : i(x), j(y), k(z) {}
  Voxel(const Eigen::Vector3d &point, double voxel_size) {
    i = static_cast<int>(point[0] / voxel_size);
    j = static_cast<int>(point[1] / voxel_size);
    k = static_cast<int>(point[2] / voxel_size);
  }
  bool operator==(const Voxel &vox) const {
    return i == vox.i && j == vox.j && k == vox.k;
  }

  int i;
  int j;
  int k;
};

}  // namespace voxel_hash_map

// Specialization of std::hash for our custom type Voxel
namespace std {

template <>
struct hash<voxel_hash_map::Voxel> {
  size_t operator()(const voxel_hash_map::Voxel &vox) const {
    const size_t kP1 = 73856093;
    const size_t kP2 = 19349669;
    const size_t kP3 = 83492791;
    return ((1 << 20) - 1) &
           (vox.i * 73856093 ^ vox.j * 19349663 ^ vox.k * 83492791);
  }
};

}  // namespace std

namespace aid4crop {
namespace utils {
std::vector<size_t> voxel_down_sample(const std::vector<Eigen::Vector3d> &,
                                      const double);
std::vector<size_t> add_to_voxel_map(
    std::unordered_map<voxel_hash_map::Voxel, uint> &,
    const std::vector<Eigen::Vector3d> &, const double);

}  // namespace utils
}  // namespace aid4crop
