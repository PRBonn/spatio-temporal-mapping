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
#include "VoxelHashMap.hpp"

namespace aid4crop {
namespace utils {

std::vector<size_t> voxel_down_sample(
    const std::vector<Eigen::Vector3d> &points, const double voxel_size) {
  // Initialization
  const size_t n_points = points.size();
  std::vector<size_t> indices;
  std::unordered_map<voxel_hash_map::Voxel, uint> voxel_map;

  // Keep only points for which we generate a new voxel
  indices.reserve(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    auto voxel = voxel_hash_map::Voxel(points[i], voxel_size);
    if (voxel_map.find(voxel) == voxel_map.end()) {
      voxel_map.insert({voxel, i});
      indices.emplace_back(i);
    }
  }
  indices.shrink_to_fit();

  return indices;
}

std::vector<size_t> add_to_voxel_map(
    std::unordered_map<voxel_hash_map::Voxel, uint> &voxel_map,
    const std::vector<Eigen::Vector3d> &points, const double voxel_size) {
  // Initialization
  const size_t n_points = points.size();
  std::vector<size_t> indices;

  // Keep only points for which we generate a new voxel
  indices.reserve(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    auto voxel = voxel_hash_map::Voxel(points[i], voxel_size);
    if (voxel_map.find(voxel) == voxel_map.end()) {
      voxel_map.insert({voxel, i});
      indices.emplace_back(i);
    }
  }
  indices.shrink_to_fit();

  return indices;
}

}  // namespace utils
}  // namespace aid4crop
