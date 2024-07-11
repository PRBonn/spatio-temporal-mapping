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

#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>

#include <eigen3/Eigen/Dense>
#include <memory>

#include "utils/voxel_hash_map/VoxelHashMap.hpp"

using PointCloud_ptr = std::shared_ptr<open3d::geometry::PointCloud>;
using PointCloud_ptr_const =
    std::shared_ptr<const open3d::geometry::PointCloud>;
using PointCloud = open3d::geometry::PointCloud;
using LineSet_ptr = std::shared_ptr<open3d::geometry::LineSet>;

namespace aid4crop {
namespace core {

class VoxelizedPointCloud {
 public:
  VoxelizedPointCloud(const double);
  VoxelizedPointCloud(const VoxelizedPointCloud& other) = delete;
  VoxelizedPointCloud(VoxelizedPointCloud&& other) = default;
  VoxelizedPointCloud& operator=(VoxelizedPointCloud& other) = delete;
  VoxelizedPointCloud& operator=(VoxelizedPointCloud&& other) = default;

  void IntegratePointCloud(const PointCloud&);
  VoxelizedPointCloud Threshold(const int, const double, const double) const;
  std::tuple<VoxelizedPointCloud, std::vector<size_t>> ThresholdWithIds(
      const int, const double, const double) const;
  VoxelizedPointCloud Resize(const double, const double) const;
  PointCloud Subsample(const double) const;
  void EstimateNormals();
  void EstimateNormals(const float&, const int);
  void addTransforms(
      const std::vector<Eigen::Transform<double, 3, Eigen::Affine>>&);
  void addSubpointsTransforms(
      const std::vector<Eigen::Transform<double, 3, Eigen::Affine>>&,
      const std::vector<size_t>&);
  void deform();
  void deform(const std::vector<Eigen::Transform<double, 3, Eigen::Affine>>&);
  void undeform();
  void replace_points(const Eigen::MatrixXd&);
  void changePointValue(const int&, const Eigen::Vector3d&);

  VoxelizedPointCloud UniformDownSample(const size_t every_k_points) const;
  VoxelizedPointCloud RemoveStatisticalOutliers(const size_t, const double,
                                                const bool = false) const;
  VoxelizedPointCloud RemoveRadiusOutliers(const size_t, const double,
                                           const bool = false) const;
  void Clean();

  size_t size() const;
  void Transform(const Eigen::Matrix4d&);
  void PaintUniformColor(const Eigen::Vector3d&);
  const PointCloud get_pcd() const;
  const PointCloud_ptr_const get_pcd_ptr() const;
  Eigen::MatrixXd get_cloud() const;
  const double get_voxel_size() const;
  std::tuple<double, double> get_limits_positions(const int) const;
  PointCloud get_moving_pcd(const float& = 0.001) const;

  const std::vector<Eigen::Vector3d>& get_points() const;
  Eigen::Vector3d get_point(const int i) const;
  std::tuple<std::vector<Eigen::Vector3d>, std::vector<size_t>>
  get_submap_points(const PointCloud&) const;

  void color_single_point(const int&, const Eigen::Vector3d&);

  void printPoint(const int&) const;
  void visualize() const;
  void visualize(const PointCloud&) const;
  void visualize(const LineSet_ptr&) const;
  void visualize_with_matches(const std::vector<Eigen::Vector3d>&,
                              const std::vector<Eigen::Vector3d>&,
                              const int sparse_factor = 1,
                              const bool change_color = true) const;
  void visualize_with_matches(const std::vector<Eigen::Vector3d>&,
                              const std::vector<Eigen::Vector3d>&,
                              const PointCloud&,
                              const double& translation = 0.0,
                              const int sparse_factor = 1,
                              const float matches_spheres_size = 0.008) const;

  static void save(const std::string&, const VoxelizedPointCloud&);
  static VoxelizedPointCloud load(const std::string&, const double);

  VoxelizedPointCloud& operator=(const VoxelizedPointCloud&);

 private:
  PointCloud_ptr _pcd;
  std::unordered_map<voxel_hash_map::Voxel, uint> _voxel_map;
  double _voxel_size;
  std::vector<Eigen::Vector3d> _points_T;
};

}  // namespace core
}  // namespace aid4crop
