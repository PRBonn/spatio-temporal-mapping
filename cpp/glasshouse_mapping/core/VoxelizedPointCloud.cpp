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
#include "VoxelizedPointCloud.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/visualization/utility/DrawGeometry.h>

#include <iostream>
#include <limits>
#include <memory>
#include <ranges>

#include "utils/voxel_hash_map/VoxelHashMap.hpp"

namespace aid4crop {
namespace core {

VoxelizedPointCloud::VoxelizedPointCloud(const double voxel_size)
    : _voxel_size(voxel_size) {
  this->_pcd = std::make_shared<open3d::geometry::PointCloud>();
  this->_points_T.resize(this->size());
  for (int i = 0; i < this->size(); ++i) {
    this->_points_T[i] = Eigen::Vector3d::Zero();
  }
};

void VoxelizedPointCloud::IntegratePointCloud(const PointCloud& other_pcd) {
  // Select only the indices of points that belong to new voxels
  auto indices_to_add = utils::add_to_voxel_map(
      this->_voxel_map, other_pcd.points_, this->_voxel_size);

  // Filter points
  PointCloud_ptr filtered_pcd = other_pcd.SelectByIndex(indices_to_add);

  // Add the new points to this pcd
  this->_pcd->points_.insert(this->_pcd->points_.end(),
                             filtered_pcd->points_.begin(),
                             filtered_pcd->points_.end());
  this->_pcd->colors_.insert(this->_pcd->colors_.end(),
                             filtered_pcd->colors_.begin(),
                             filtered_pcd->colors_.end());
  this->_points_T.insert(this->_points_T.end(), indices_to_add.size(),
                         Eigen::Vector3d::Zero());
}

VoxelizedPointCloud VoxelizedPointCloud::Threshold(const int axis,
                                                   const double min_th,
                                                   const double max_th) const {
  // Initialization
  VoxelizedPointCloud output_voxelized_pcd(this->_voxel_size);
  const uint n_points = this->size();
  std::vector<size_t> indices;

  // Get only the needed points indices.reserve(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    const auto& el = this->_pcd->points_[i][axis];
    if (el > min_th && el < max_th) {
      indices.emplace_back(i);
    }
  }

  output_voxelized_pcd.IntegratePointCloud(*this->_pcd->SelectByIndex(indices));
  return std::move(output_voxelized_pcd);
}

VoxelizedPointCloud VoxelizedPointCloud::Resize(const double x_center,
                                                const double dimension) const {
  return this->Threshold(0, x_center - dimension, x_center + dimension);
}

std::tuple<VoxelizedPointCloud, std::vector<size_t>>
VoxelizedPointCloud::ThresholdWithIds(const int axis, const double min_th,
                                      const double max_th) const {
  // Initialization
  VoxelizedPointCloud output_voxelized_pcd(this->_voxel_size);
  const uint n_points = this->size();
  std::vector<size_t> indices;

  // Get only the needed points indices.reserve(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    const auto& el = this->_pcd->points_[i][axis];
    if (el > min_th && el < max_th) {
      indices.emplace_back(i);
    }
  }

  output_voxelized_pcd.IntegratePointCloud(*this->_pcd->SelectByIndex(indices));
  return std::make_tuple(std::move(output_voxelized_pcd), indices);
}

void VoxelizedPointCloud::addTransforms(
    const std::vector<Eigen::Transform<double, 3, Eigen::Affine>>& points_T) {
  assert(points_T.size() == this->size());
  for (int i = 0; i < this->size(); ++i) {
    Eigen::Vector3d new_pt_position = points_T[i] * this->_pcd->points_[i];
    Eigen::Vector3d new_t = new_pt_position - this->_pcd->points_[i];
    if (this->_points_T[i] != Eigen::Vector3d::Zero()) {
      this->_points_T[i] = (this->_points_T[i] + new_t) / 2.0;
    } else {
      this->_points_T[i] = new_t;
    }
  }
}

void VoxelizedPointCloud::addSubpointsTransforms(
    const std::vector<Eigen::Transform<double, 3, Eigen::Affine>>& points_T,
    const std::vector<size_t>& sub_indices) {
  assert(points_T.size() == sub_indices.size());
  for (int i = 0; i < sub_indices.size(); ++i) {
    Eigen::Vector3d new_pt_position =
        points_T[i] * this->_pcd->points_[sub_indices[i]];
    Eigen::Vector3d new_t =
        new_pt_position - this->_pcd->points_[sub_indices[i]];
    /* if (new_t.norm() > 0.06f) continue; */
    if (this->_points_T[sub_indices[i]] != Eigen::Vector3d::Zero()) {
      this->_points_T[sub_indices[i]] =
          (this->_points_T[sub_indices[i]] + new_t) / 2.0;
    } else {
      this->_points_T[sub_indices[i]] = new_t;
    }
  }
}

void VoxelizedPointCloud::deform(
    const std::vector<Eigen::Transform<double, 3, Eigen::Affine>>& points_T) {
  assert(points_T.size() == this->size());
  for (int i = 0; i < this->size(); ++i) {
    this->_pcd->points_[i] = points_T[i] * this->_pcd->points_[i];
    this->_points_T[i] = points_T[i] * this->_points_T[i];
  }
}

void VoxelizedPointCloud::deform() {
  for (int i = 0; i < this->size(); ++i) {
    this->_pcd->points_[i] += this->_points_T[i];
  }
}

void VoxelizedPointCloud::undeform() {
  for (int i = 0; i < this->size(); ++i) {
    this->_pcd->points_[i] -= this->_points_T[i];
  }
}

void VoxelizedPointCloud::replace_points(const Eigen::MatrixXd& new_positions) {
  assert(this->size() == new_positions.rows());
  for (int pt_idx = 0; pt_idx < new_positions.rows(); ++pt_idx) {
    this->_pcd->points_[pt_idx] = new_positions.row(pt_idx);
  }
}

void VoxelizedPointCloud::changePointValue(const int& pt_idx,
                                           const Eigen::Vector3d& new_pt) {
  assert(pt_idx >= 0 && pt_idx < this->size());
  this->_pcd->points_[pt_idx] = new_pt;
}

VoxelizedPointCloud VoxelizedPointCloud::UniformDownSample(
    const size_t every_k_points) const {
  VoxelizedPointCloud downsampled_pcd = VoxelizedPointCloud(this->_voxel_size);
  downsampled_pcd.IntegratePointCloud(
      *(this->_pcd->UniformDownSample(every_k_points)));
  return downsampled_pcd;
}

VoxelizedPointCloud VoxelizedPointCloud::RemoveStatisticalOutliers(
    const size_t nb_neighbors, const double std_ratio,
    const bool print_progress) const {
  VoxelizedPointCloud clean_pcd = VoxelizedPointCloud(this->_voxel_size);
  clean_pcd.IntegratePointCloud(
      *(std::get<0>(this->_pcd->RemoveStatisticalOutliers(
          nb_neighbors, std_ratio, print_progress))));
  return clean_pcd;
}

VoxelizedPointCloud VoxelizedPointCloud::RemoveRadiusOutliers(
    const size_t nb_points, const double search_radius,
    const bool print_progress) const {
  VoxelizedPointCloud clean_pcd = VoxelizedPointCloud(this->_voxel_size);
  clean_pcd.IntegratePointCloud(*std::get<0>(this->_pcd->RemoveRadiusOutliers(
      nb_points, search_radius, print_progress)));
  return clean_pcd;
}

void VoxelizedPointCloud::Clean() {
  this->_pcd->points_ = {};
  this->_pcd->colors_ = {};
  this->_points_T = {};
}

size_t VoxelizedPointCloud::size() const { return this->_pcd->points_.size(); }

void VoxelizedPointCloud::Transform(const Eigen::Matrix4d& T) {
  this->_pcd->Transform(T);
}

void VoxelizedPointCloud::PaintUniformColor(const Eigen::Vector3d& color) {
  this->_pcd->PaintUniformColor(color);
}

const PointCloud VoxelizedPointCloud::get_pcd() const { return *this->_pcd; }

const PointCloud_ptr_const VoxelizedPointCloud::get_pcd_ptr() const {
  return this->_pcd;
}

Eigen::MatrixXd VoxelizedPointCloud::get_cloud() const {
  auto cloud = Eigen::MatrixXd(this->size(), 3);
  const auto& points = this->get_pcd().points_;
  for (int pt_idx = 0; pt_idx < points.size(); ++pt_idx) {
    cloud.row(pt_idx) = points[pt_idx];
  }
  return cloud;
}

const double VoxelizedPointCloud::get_voxel_size() const {
  return this->_voxel_size;
}

std::tuple<double, double> VoxelizedPointCloud::get_limits_positions(
    const int axis) const {
  double min_val = std::numeric_limits<double>::infinity();
  double max_val = -std::numeric_limits<double>::infinity();
  for (const auto& p : this->_pcd->points_) {
    if (p[axis] < min_val) min_val = p[axis];
    if (p[axis] > max_val) max_val = p[axis];
  }
  return {min_val, max_val};
}

PointCloud VoxelizedPointCloud::get_moving_pcd(const float& moving_th) const {
  // Initialization
  PointCloud moving_pcd;
  const size_t n_points = this->size();

  for (int i = 0; i < n_points; ++i) {
    if (this->_points_T[i].norm() > moving_th) {
      moving_pcd.points_.emplace_back(this->_pcd->points_[i]);
      moving_pcd.colors_.emplace_back(this->_pcd->colors_[i]);
    }
  }

  return moving_pcd;
}

const std::vector<Eigen::Vector3d>& VoxelizedPointCloud::get_points() const {
  return (*this->_pcd).points_;
}

Eigen::Vector3d VoxelizedPointCloud::get_point(const int i) const {
  assert(i > 0 && i < this->size());
  return this->_pcd->points_[i];
}
/*
 * This function takes a point cloud and returns the list of points in this
 * that are inside the range <min_x, max_x> of the given point cloud, where
 * min_x and max_x are the min and max values of the points in the given
 * point cloud. It also returns a list of indices that maps the new points
 * to the set of the un-filtered one.
 */
std::tuple<std::vector<Eigen::Vector3d>, std::vector<size_t>>
VoxelizedPointCloud::get_submap_points(const PointCloud& pcd_to_fit) const {
  // Initialization
  std::vector<Eigen::Vector3d> sub_points;
  std::vector<size_t> sub_indices;
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();

  // Compute the min and max values of the pcd to fit
  for (const auto& p : pcd_to_fit.points_) {
    if (p[0] < min_x) min_x = p[0];
    if (p[0] > max_x) max_x = p[0];
  }

  // Get only the needed points indices.reserve(n_points);
  const size_t n_points_this = this->size();
  for (size_t i = 0; i < n_points_this; ++i) {
    const auto& el = this->_pcd->points_[i][0];
    if (el > min_x && el < max_x) {
      sub_indices.emplace_back(i);
    }
  }

  // Copy the thresholded points
  const size_t n_sub_points = sub_indices.size();
  sub_points.resize(n_sub_points);
  for (size_t i = 0; i < n_sub_points; ++i) {
    sub_points[i] = this->_pcd->points_[sub_indices[i]];
  }

  return {sub_points, sub_indices};
}

void VoxelizedPointCloud::color_single_point(const int& point_idx,
                                             const Eigen::Vector3d& color) {
  assert(point_idx >= 0 && point_idx < this->size());
  this->_pcd->colors_[point_idx] = color;
}

void VoxelizedPointCloud::printPoint(const int& pt_idx) const {
  assert(pt_idx >= 0 && pt_idx < this->size());
  std::cout << this->_pcd->points_[pt_idx] << std::endl;
}

void VoxelizedPointCloud::visualize() const {
  open3d::visualization::DrawGeometries({this->_pcd});
}

void VoxelizedPointCloud::visualize(const PointCloud& other_pcd) const {
  // Initialization
  const Eigen::Vector3d source_pcd_color =
      Eigen::Vector3d(97, 137, 133) / 255.0;
  const Eigen::Vector3d target_pcd_color =
      Eigen::Vector3d(193, 152, 117) / 255.0;

  // Change the color of the two point clouds
  PointCloud source_pcd = *this->_pcd;
  /* source_pcd.PaintUniformColor(source_pcd_color); */
  PointCloud target_pcd = other_pcd;
  target_pcd.PaintUniformColor(target_pcd_color);

  // Visualize
  open3d::visualization::DrawGeometries(
      {std::make_shared<PointCloud>(source_pcd),
       std::make_shared<PointCloud>(target_pcd)});
}

void VoxelizedPointCloud::visualize(const LineSet_ptr& lineSet_ptr) const {
  open3d::visualization::DrawGeometries({this->_pcd, lineSet_ptr});
}

// Function to visualize only the reference pcd with lines (matches)
void VoxelizedPointCloud::visualize_with_matches(
    const std::vector<Eigen::Vector3d>& source_pts,
    const std::vector<Eigen::Vector3d>& target_pts, const int sparse_factor,
    const bool change_color) const {
  // Initialization
  assert(source_pts.size() == target_pts.size());
  const Eigen::Vector3d source_pcd_color =
      Eigen::Vector3d(97, 137, 133) / 255.0;
  const Eigen::Vector3d target_pcd_color =
      Eigen::Vector3d(193, 152, 117) / 255.0;
  const Eigen::Vector3d lines_color = Eigen::Vector3d(186, 27, 29) / 255.0;
  const Eigen::Vector3d endpoints_color = Eigen::Vector3d(186, 27, 29) / 255.0;

  // Create a open3d version of the points
  auto matches_pcd = open3d::geometry::PointCloud();
  for (int i = 0; i < source_pts.size(); ++i) {
    matches_pcd.points_.emplace_back(source_pts[i]);
    matches_pcd.colors_.emplace_back(endpoints_color);
    matches_pcd.points_.emplace_back(target_pts[i]);
    matches_pcd.colors_.emplace_back(endpoints_color);
  }

  // Create a line for each match
  std::vector<Eigen::Vector2i> lines;
  for (int r = 0; r < matches_pcd.points_.size(); r += 2 * sparse_factor) {
    lines.emplace_back(Eigen::Vector2i(r, r + 1));
  }
  auto lineSet_ptr =
      std::make_shared<open3d::geometry::LineSet>(matches_pcd.points_, lines);
  lineSet_ptr->PaintUniformColor(lines_color);

  // Change the colors of the pcd to visualize if requested
  PointCloud source_pcd = *this->_pcd;
  if (change_color) {
    source_pcd.PaintUniformColor(source_pcd_color);
  }

  // Visualize
  open3d::visualization::DrawGeometries(
      {std::make_shared<PointCloud>(source_pcd),
       std::make_shared<PointCloud>(matches_pcd), lineSet_ptr});
}

// Function to visualize the reference pcd togheter with the query pcd and
// relative matches
void VoxelizedPointCloud::visualize_with_matches(
    const std::vector<Eigen::Vector3d>& source_pts,
    const std::vector<Eigen::Vector3d>& target_pts, const PointCloud& other_pcd,
    const double& translation, const int sparse_factor,
    const float matches_spheres_size) const {
  // Initialization
  assert(source_pts.size() == target_pts.size());
  const Eigen::Vector3d source_pcd_color =
      Eigen::Vector3d(97, 137, 133) / 255.0;
  const Eigen::Vector3d target_pcd_color =
      Eigen::Vector3d(193, 152, 117) / 255.0;
  const Eigen::Vector3d lines_color = Eigen::Vector3d(65, 69, 53) / 255.0;
  const Eigen::Vector3d endpoints_color = Eigen::Vector3d(186, 27, 29) / 255.0;

  // Create a sphere in each matched point
  std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> matches_spheres;
  matches_spheres.reserve(
      std::ceil((source_pts.size() / (float)sparse_factor) * 2.0));
  auto matches_pcd = open3d::geometry::PointCloud();
  for (int i = 0; i < source_pts.size(); i += sparse_factor) {
    // Translate the target points by some amount
    auto trg_pt = target_pts[i];
    trg_pt[0] = trg_pt[0] + translation;

    // Create the source sphere
    std::shared_ptr<open3d::geometry::TriangleMesh> source_sphere =
        open3d::geometry::TriangleMesh::CreateSphere(matches_spheres_size);
    source_sphere->Translate(source_pts[i]);
    source_sphere->PaintUniformColor(endpoints_color);
    matches_spheres.emplace_back(source_sphere);

    // Create the target sphere
    std::shared_ptr<open3d::geometry::TriangleMesh> target_sphere =
        open3d::geometry::TriangleMesh::CreateSphere(matches_spheres_size);
    target_sphere->Translate(trg_pt);
    target_sphere->PaintUniformColor(endpoints_color);
    matches_spheres.emplace_back(target_sphere);

    // Create the pcd of the matches (useful for lines)
    matches_pcd.points_.emplace_back(source_pts[i]);
    matches_pcd.colors_.emplace_back(endpoints_color);
    matches_pcd.points_.emplace_back(trg_pt);
    matches_pcd.colors_.emplace_back(endpoints_color);
  }

  // Create a line for each match
  std::vector<Eigen::Vector2i> lines;
  for (int r = 0; r < matches_pcd.points_.size(); r += 2) {
    lines.emplace_back(Eigen::Vector2i(r, r + 1));
  }
  auto lineSet_ptr =
      std::make_shared<open3d::geometry::LineSet>(matches_pcd.points_, lines);
  lineSet_ptr->PaintUniformColor(lines_color);

  // Change the colors of the pcds to visualize
  PointCloud source_pcd = *this->_pcd;
  /* source_pcd.PaintUniformColor(source_pcd_color); */
  PointCloud target_pcd = other_pcd;
  target_pcd.PaintUniformColor(target_pcd_color);

  // Translate also the other pcd
  for (auto& pt : target_pcd.points_) {
    pt = pt + Eigen::Vector3d(translation, 0.0, 0.0);
  }

  // Visualize everything
  std::vector<std::shared_ptr<const open3d::geometry::Geometry>>
      geometries_to_visualize;
  geometries_to_visualize.emplace_back(
      std::make_shared<PointCloud>(source_pcd));
  geometries_to_visualize.emplace_back(lineSet_ptr);
  geometries_to_visualize.emplace_back(
      std::make_shared<PointCloud>(target_pcd));
  geometries_to_visualize.insert(geometries_to_visualize.end(),
                                 matches_spheres.begin(),
                                 matches_spheres.end());
  open3d::visualization::DrawGeometries(geometries_to_visualize);
}

PointCloud VoxelizedPointCloud::Subsample(const double voxel_size) const {
  std::unordered_map<voxel_hash_map::Voxel, uint> voxel_map;
  auto indices =
      utils::add_to_voxel_map(voxel_map, this->_pcd->points_, voxel_size);
  return *this->_pcd->SelectByIndex(indices);
}

void VoxelizedPointCloud::EstimateNormals() {
  // Invalidate existing normals
  this->_pcd->normals_ =
      std::vector<Eigen::Vector3d>(this->size(), Eigen::Vector3d::Zero());

  // Compute normals and make their orientation consistent with the neighbors
  this->_pcd->EstimateNormals();
  this->_pcd->OrientNormalsConsistentTangentPlane(100);
}

void VoxelizedPointCloud::EstimateNormals(const float& radius_normal,
                                          const int max_nn) {
  // Invalidate existing normals
  this->_pcd->normals_ =
      std::vector<Eigen::Vector3d>(this->size(), Eigen::Vector3d::Zero());

  // Compute normals and make their orientation consistent with the neighbors
  this->_pcd->EstimateNormals(
      open3d::geometry::KDTreeSearchParamHybrid(radius_normal, max_nn));
}

void VoxelizedPointCloud::save(const std::string& filename,
                               const VoxelizedPointCloud& voxelized_pcd) {
  // Save the point cloud
  open3d::io::WritePointCloud(filename, *voxelized_pcd._pcd);
}

VoxelizedPointCloud VoxelizedPointCloud::load(const std::string& filename,
                                              const double voxel_size) {
  // Initialization
  auto voxelized_pcd = VoxelizedPointCloud(voxel_size);

  // Load the pcd
  auto pcd = open3d::geometry::PointCloud();
  open3d::io::ReadPointCloud(filename, pcd);

  // Add the pcd to the voxelized pcd
  voxelized_pcd.IntegratePointCloud(pcd);

  return voxelized_pcd;
}

VoxelizedPointCloud& VoxelizedPointCloud::operator=(
    const VoxelizedPointCloud& other) {
  // Self-assignment check
  if (this == &other) {
    return *this;
  }

  _pcd = other._pcd;
  _voxel_size = other._voxel_size;
  _voxel_map = other._voxel_map;

  return *this;
}

}  // namespace core
}  // namespace aid4crop
