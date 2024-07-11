// ----------------------------------------------------------------------------
// NOTE: This file has been adapted from the following project, but copyright
// still belongs to them. All rights reserved
// ----------------------------------------------------------------------------
// -            https://github.com/rFalque/embedded_deformation               -
// ----------------------------------------------------------------------------
// MIT License
//
// Copyright (c) 2018 Raphael Falque
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

#include <ceres/ceres.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <memory>
#include <vector>

#include "VoxelizedPointCloud.hpp"
#include "utils/libGraphCpp/graph.hpp"
#include "utils/nanoflann/nanoflannWrapper.hpp"

namespace aid4crop {
namespace core {

struct three_d_point {
  double x;
  double y;
  double z;
};

struct point_and_occurences {
  double x;
  double y;
  double z;
  int occurence;
};

class DeformationGraph {
 public:
  DeformationGraph(const VoxelizedPointCloud&, const float&, const int);

  Eigen::MatrixXd deform(
      const std::vector<Eigen::Vector3d>&, const std::vector<Eigen::Vector3d>&,
      const double = -std::numeric_limits<double>::infinity(),
      const double = std::numeric_limits<double>::infinity());

  std::vector<Eigen::Vector3d> get_points() const;
  Eigen::MatrixXd get_cloud() const;

  size_t size() const;
  void visualize() const;
  void visualize(const DeformationGraph&) const;

 private:
  Eigen::MatrixXd _cloud;  // The original cloud
  Eigen::MatrixXd _nodes;
  Eigen::MatrixXi _edges;
  nanoflann_wrapper _nodes_kdtree;
  nanoflann_wrapper _cloud_kdtree;
  std::shared_ptr<libgraphcpp::Graph> _deformation_graph_ptr;
  std::vector<int> _node_to_cloud_ids;  // In position i contains the id of the
                                        // node i in _cloud
  const float _resolution;
  const int _nodes_connectivity;
  const double _cloud_voxel_size;

  static inline void getMinMax(const Eigen::MatrixXd& in_cloud,
                               Eigen::Vector3d& min_point,
                               Eigen::Vector3d& max_point) {
    max_point = in_cloud.colwise().maxCoeff();
    min_point = in_cloud.colwise().minCoeff();
  };

  inline void getScale(const Eigen::MatrixXd& in_cloud, double& scale) {
    Eigen::Vector3d min_point;
    Eigen::Vector3d max_point;

    getMinMax(in_cloud, min_point, max_point);

    scale = (max_point - min_point).norm();
  };

  inline void voxel_grid_downsampling(Eigen::MatrixXd& in_cloud,
                                      double leaf_size,
                                      Eigen::MatrixXd& out_cloud) {
    Eigen::Vector3d min_point, max_point;
    getMinMax(in_cloud, min_point, max_point);

    double inv_leaf_size;
    inv_leaf_size = 1.0 / leaf_size;

    Eigen::Vector3i min_box, max_box;
    min_box << floor(min_point(0) * inv_leaf_size),
        floor(min_point(1) * inv_leaf_size),
        floor(min_point(2) * inv_leaf_size);
    max_box << floor(max_point(0) * inv_leaf_size),
        floor(max_point(1) * inv_leaf_size),
        floor(max_point(2) * inv_leaf_size);

    Eigen::Vector3i divb, divb_mul;
    divb << max_box(0) - min_box(0) + 1, max_box(1) - min_box(1) + 1,
        max_box(2) - min_box(2) + 1;
    divb_mul << 1, divb(0), divb(0) * divb(1);

    std::vector<std::vector<std::vector<point_and_occurences>>> voxels;

    voxels.resize(divb(0));
    for (int x_index = 0; x_index < voxels.size(); ++x_index) {
      voxels[x_index].resize(divb(1));
      for (int y_index = 0; y_index < voxels[0].size(); ++y_index) {
        voxels[x_index][y_index].resize(divb(2));
      }
    }

    // plus assign zeros to voxel_count
    for (int i = 0; i < in_cloud.rows(); ++i) {
      int x_index =
          static_cast<int>(floor(in_cloud(i, 0) * inv_leaf_size) - min_box(0));
      int y_index =
          static_cast<int>(floor(in_cloud(i, 1) * inv_leaf_size) - min_box(1));
      int z_index =
          static_cast<int>(floor(in_cloud(i, 2) * inv_leaf_size) - min_box(2));

      voxels[x_index][y_index][z_index].x += in_cloud(i, 0);
      voxels[x_index][y_index][z_index].y += in_cloud(i, 1);
      voxels[x_index][y_index][z_index].z += in_cloud(i, 2);
      voxels[x_index][y_index][z_index].occurence++;
    }

    std::vector<three_d_point> final_cloud;
    three_d_point temp;
    for (int x_index = 0; x_index < voxels.size(); ++x_index) {
      for (int y_index = 0; y_index < voxels[0].size(); ++y_index) {
        for (int z_index = 0; z_index < voxels[0][0].size(); ++z_index) {
          if (voxels[x_index][y_index][z_index].occurence != 0) {
            temp.x = voxels[x_index][y_index][z_index].x /
                     voxels[x_index][y_index][z_index].occurence;
            temp.y = voxels[x_index][y_index][z_index].y /
                     voxels[x_index][y_index][z_index].occurence;
            temp.z = voxels[x_index][y_index][z_index].z /
                     voxels[x_index][y_index][z_index].occurence;
            final_cloud.push_back(temp);
          }
        }
      }
    }

    out_cloud.resize(final_cloud.size(), 3);
    for (int i = 0; i < final_cloud.size(); ++i) {
      out_cloud.row(i) << final_cloud[i].x, final_cloud[i].y, final_cloud[i].z;
    }
  };

  inline std::tuple<Eigen::MatrixXd, std::vector<int>> downsampling(
      Eigen::MatrixXd& in_cloud, double leaf_size) {
    // Initialization
    Eigen::MatrixXd downsampled_cloud, out_cloud;
    std::vector<int> in_cloud_samples;

    // Downsample the cloud
    voxel_grid_downsampling(in_cloud, leaf_size, downsampled_cloud);

    // Fill the output
    out_cloud.resize(downsampled_cloud.rows(), 3);
    nanoflann_wrapper tree(in_cloud);
    for (int i = 0; i < downsampled_cloud.rows(); ++i) {
      std::vector<int> closest_point;
      closest_point = tree.return_k_closest_points(downsampled_cloud.row(i), 1);

      out_cloud.row(i) = in_cloud.row(closest_point[0]);
      in_cloud_samples.push_back(closest_point[0]);
    }

    return {out_cloud, in_cloud_samples};
  };
};

// ---------------------------------------------------------------------------------------------------
// --- Cost functions ---
// ---------------------------------------------------------------------------------------------------

// norm2(R'*R-I)^2
class RotCostFunction : public ceres::CostFunction {
 public:
  // constructor
  RotCostFunction(double cost_function_weight) {
    // define residual and block_size
    set_num_residuals(9);
    std::vector<int>* block_sizes = mutable_parameter_block_sizes();
    block_sizes->push_back(12);

    cost_function_weight_ = sqrt(cost_function_weight);
  }

  ~RotCostFunction() {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    Eigen::Map<const Eigen::Matrix3d> R(parameters[0]);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);

    Eigen::Map<Eigen::MatrixXd> res(residuals, 3, 3);
    res = cost_function_weight_ * (R.transpose() * R - I);

    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        // R'*R - I (0,0)
        jacobians[0][0] = cost_function_weight_ * 2 * R(0, 0);  // df/dR(0,0)
        jacobians[0][1] = cost_function_weight_ * 2 * R(1, 0);  // df/dR(1,0)
        jacobians[0][2] = cost_function_weight_ * 2 * R(2, 0);  // df/dR(2,0)
        jacobians[0][3] = 0;                                    // df/dR(0,1)
        jacobians[0][4] = 0;                                    // df/dR(1,1)
        jacobians[0][5] = 0;                                    // df/dR(2,1)
        jacobians[0][6] = 0;                                    // df/dR(0,2)
        jacobians[0][7] = 0;                                    // df/dR(1,2)
        jacobians[0][8] = 0;                                    // df/dR(2,2)
        jacobians[0][9] = 0;                                    // df/dT(0)
        jacobians[0][10] = 0;                                   // df/dT(1)
        jacobians[0][11] = 0;                                   // df/dT(2)

        // R'*R - I (0,1)
        jacobians[0][0 + 12] = cost_function_weight_ * R(0, 1);
        jacobians[0][1 + 12] = cost_function_weight_ * R(1, 1);
        jacobians[0][2 + 12] = cost_function_weight_ * R(2, 1);
        jacobians[0][3 + 12] = cost_function_weight_ * R(0, 0);
        jacobians[0][4 + 12] = cost_function_weight_ * R(1, 0);
        jacobians[0][5 + 12] = cost_function_weight_ * R(2, 0);
        jacobians[0][6 + 12] = 0;
        jacobians[0][7 + 12] = 0;
        jacobians[0][8 + 12] = 0;
        jacobians[0][9 + 12] = 0;
        jacobians[0][10 + 12] = 0;
        jacobians[0][11 + 12] = 0;

        // R'*R - I (0,2)
        jacobians[0][0 + 2 * 12] = cost_function_weight_ * R(0, 2);
        jacobians[0][1 + 2 * 12] = cost_function_weight_ * R(1, 2);
        jacobians[0][2 + 2 * 12] = cost_function_weight_ * R(2, 2);
        jacobians[0][3 + 2 * 12] = 0;
        jacobians[0][4 + 2 * 12] = 0;
        jacobians[0][5 + 2 * 12] = 0;
        jacobians[0][6 + 2 * 12] = cost_function_weight_ * R(0, 0);
        jacobians[0][7 + 2 * 12] = cost_function_weight_ * R(1, 0);
        jacobians[0][8 + 2 * 12] = cost_function_weight_ * R(2, 0);
        jacobians[0][9 + 2 * 12] = 0;
        jacobians[0][10 + 2 * 12] = 0;
        jacobians[0][11 + 2 * 12] = 0;

        // R'*R - I (1,0)
        jacobians[0][0 + 3 * 12] = cost_function_weight_ * R(0, 1);
        jacobians[0][1 + 3 * 12] = cost_function_weight_ * R(1, 1);
        jacobians[0][2 + 3 * 12] = cost_function_weight_ * R(2, 1);
        jacobians[0][3 + 3 * 12] = cost_function_weight_ * R(0, 0);
        jacobians[0][4 + 3 * 12] = cost_function_weight_ * R(1, 0);
        jacobians[0][5 + 3 * 12] = cost_function_weight_ * R(2, 0);
        jacobians[0][6 + 3 * 12] = 0;
        jacobians[0][7 + 3 * 12] = 0;
        jacobians[0][8 + 3 * 12] = 0;
        jacobians[0][9 + 3 * 12] = 0;
        jacobians[0][10 + 3 * 12] = 0;
        jacobians[0][11 + 3 * 12] = 0;

        // R'*R - I (1,1)
        jacobians[0][0 + 4 * 12] = 0;
        jacobians[0][1 + 4 * 12] = 0;
        jacobians[0][2 + 4 * 12] = 0;
        jacobians[0][3 + 4 * 12] = cost_function_weight_ * 2 * R(0, 1);
        jacobians[0][4 + 4 * 12] = cost_function_weight_ * 2 * R(1, 1);
        jacobians[0][5 + 4 * 12] = cost_function_weight_ * 2 * R(2, 1);
        jacobians[0][6 + 4 * 12] = 0;
        jacobians[0][7 + 4 * 12] = 0;
        jacobians[0][8 + 4 * 12] = 0;
        jacobians[0][9 + 4 * 12] = 0;
        jacobians[0][10 + 4 * 12] = 0;
        jacobians[0][11 + 4 * 12] = 0;

        // R'*R - I (1,2)
        jacobians[0][0 + 5 * 12] = 0;
        jacobians[0][1 + 5 * 12] = 0;
        jacobians[0][2 + 5 * 12] = 0;
        jacobians[0][3 + 5 * 12] = cost_function_weight_ * R(0, 2);
        jacobians[0][4 + 5 * 12] = cost_function_weight_ * R(1, 2);
        jacobians[0][5 + 5 * 12] = cost_function_weight_ * R(2, 2);
        jacobians[0][6 + 5 * 12] = cost_function_weight_ * R(0, 1);
        jacobians[0][7 + 5 * 12] = cost_function_weight_ * R(1, 1);
        jacobians[0][8 + 5 * 12] = cost_function_weight_ * R(2, 1);
        jacobians[0][9 + 5 * 12] = 0;
        jacobians[0][10 + 5 * 12] = 0;
        jacobians[0][11 + 5 * 12] = 0;

        // R'*R - I (2,0)
        jacobians[0][0 + 6 * 12] = cost_function_weight_ * R(0, 2);
        jacobians[0][1 + 6 * 12] = cost_function_weight_ * R(1, 2);
        jacobians[0][2 + 6 * 12] = cost_function_weight_ * R(2, 2);
        jacobians[0][3 + 6 * 12] = 0;
        jacobians[0][4 + 6 * 12] = 0;
        jacobians[0][5 + 6 * 12] = 0;
        jacobians[0][6 + 6 * 12] = cost_function_weight_ * R(0, 0);
        jacobians[0][7 + 6 * 12] = cost_function_weight_ * R(1, 0);
        jacobians[0][8 + 6 * 12] = cost_function_weight_ * R(2, 0);
        jacobians[0][9 + 6 * 12] = 0;
        jacobians[0][10 + 6 * 12] = 0;
        jacobians[0][11 + 6 * 12] = 0;

        // R'*R - I (2,1)
        jacobians[0][0 + 7 * 12] = 0;
        jacobians[0][1 + 7 * 12] = 0;
        jacobians[0][2 + 7 * 12] = 0;
        jacobians[0][3 + 7 * 12] = cost_function_weight_ * R(0, 2);
        jacobians[0][4 + 7 * 12] = cost_function_weight_ * R(1, 2);
        jacobians[0][5 + 7 * 12] = cost_function_weight_ * R(2, 2);
        jacobians[0][6 + 7 * 12] = cost_function_weight_ * R(0, 1);
        jacobians[0][7 + 7 * 12] = cost_function_weight_ * R(1, 1);
        jacobians[0][8 + 7 * 12] = cost_function_weight_ * R(2, 1);
        jacobians[0][9 + 7 * 12] = 0;
        jacobians[0][10 + 7 * 12] = 0;
        jacobians[0][11 + 7 * 12] = 0;

        // R'*R - I (2,2)
        jacobians[0][0 + 8 * 12] = 0;
        jacobians[0][1 + 8 * 12] = 0;
        jacobians[0][2 + 8 * 12] = 0;
        jacobians[0][3 + 8 * 12] = 0;
        jacobians[0][4 + 8 * 12] = 0;
        jacobians[0][5 + 8 * 12] = 0;
        jacobians[0][6 + 8 * 12] = cost_function_weight_ * 2 * R(0, 2);
        jacobians[0][7 + 8 * 12] = cost_function_weight_ * 2 * R(1, 2);
        jacobians[0][8 + 8 * 12] = cost_function_weight_ * 2 * R(2, 2);
        jacobians[0][9 + 8 * 12] = 0;
        jacobians[0][10 + 8 * 12] = 0;
        jacobians[0][11 + 8 * 12] = 0;
      }
    }
    return true;
  }

 private:
  double cost_function_weight_;
};

class RegCostFunction : public ceres::CostFunction {
 public:
  // constructor
  RegCostFunction(double cost_function_weight, Eigen::Vector3d g_j,
                  Eigen::Vector3d g_k) {
    // define residual and block_size
    set_num_residuals(3);
    std::vector<int>* block_sizes = mutable_parameter_block_sizes();
    block_sizes->push_back(12);
    block_sizes->push_back(12);

    g_j_ = g_j;
    g_k_ = g_k;
    cost_function_weight_ = sqrt(cost_function_weight);
  }

  ~RegCostFunction() {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    Eigen::Map<const Eigen::Matrix3d> R_j(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_j(parameters[0] + 9);
    Eigen::Map<const Eigen::Matrix3d> R_k(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> t_k(parameters[1] + 9);

    Eigen::Map<Eigen::Vector3d> res(residuals, 3);

    double alpha_j_k = 1;

    res = cost_function_weight_ * alpha_j_k *
          (R_j * (g_k_ - g_j_) + g_j_ + t_j - (g_k_ + t_k));

    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        jacobians[0][0] =
            cost_function_weight_ * alpha_j_k * (g_k_(0) - g_j_(0));
        jacobians[0][1] = 0;
        jacobians[0][2] = 0;
        jacobians[0][3] =
            cost_function_weight_ * alpha_j_k * (g_k_(1) - g_j_(1));
        jacobians[0][4] = 0;
        jacobians[0][5] = 0;
        jacobians[0][6] =
            cost_function_weight_ * alpha_j_k * (g_k_(2) - g_j_(2));
        jacobians[0][7] = 0;
        jacobians[0][8] = 0;
        jacobians[0][9] = cost_function_weight_ * alpha_j_k;
        jacobians[0][10] = 0;
        jacobians[0][11] = 0;

        jacobians[0][0 + 12] = 0;
        jacobians[0][1 + 12] =
            cost_function_weight_ * alpha_j_k * (g_k_(0) - g_j_(0));
        jacobians[0][2 + 12] = 0;
        jacobians[0][3 + 12] = 0;
        jacobians[0][4 + 12] =
            cost_function_weight_ * alpha_j_k * (g_k_(1) - g_j_(1));
        jacobians[0][5 + 12] = 0;
        jacobians[0][6 + 12] = 0;
        jacobians[0][7 + 12] =
            cost_function_weight_ * alpha_j_k * (g_k_(2) - g_j_(2));
        jacobians[0][8 + 12] = 0;
        jacobians[0][9 + 12] = 0;
        jacobians[0][10 + 12] = cost_function_weight_ * alpha_j_k;
        jacobians[0][11 + 12] = 0;

        jacobians[0][0 + 12 * 2] = 0;
        jacobians[0][1 + 12 * 2] = 0;
        jacobians[0][2 + 12 * 2] =
            cost_function_weight_ * alpha_j_k * (g_k_(0) - g_j_(0));
        jacobians[0][3 + 12 * 2] = 0;
        jacobians[0][4 + 12 * 2] = 0;
        jacobians[0][5 + 12 * 2] =
            cost_function_weight_ * alpha_j_k * (g_k_(1) - g_j_(1));
        jacobians[0][6 + 12 * 2] = 0;
        jacobians[0][7 + 12 * 2] = 0;
        jacobians[0][8 + 12 * 2] =
            cost_function_weight_ * alpha_j_k * (g_k_(2) - g_j_(2));
        jacobians[0][9 + 12 * 2] = 0;
        jacobians[0][10 + 12 * 2] = 0;
        jacobians[0][11 + 12 * 2] = cost_function_weight_ * alpha_j_k;

        jacobians[1][0] = 0;
        jacobians[1][1] = 0;
        jacobians[1][2] = 0;
        jacobians[1][3] = 0;
        jacobians[1][4] = 0;
        jacobians[1][5] = 0;
        jacobians[1][6] = 0;
        jacobians[1][7] = 0;
        jacobians[1][8] = 0;
        jacobians[1][9] = -cost_function_weight_ * alpha_j_k;
        jacobians[1][10] = 0;
        jacobians[1][11] = 0;

        jacobians[1][0 + 12] = 0;
        jacobians[1][1 + 12] = 0;
        jacobians[1][2 + 12] = 0;
        jacobians[1][3 + 12] = 0;
        jacobians[1][4 + 12] = 0;
        jacobians[1][5 + 12] = 0;
        jacobians[1][6 + 12] = 0;
        jacobians[1][7 + 12] = 0;
        jacobians[1][8 + 12] = 0;
        jacobians[1][9 + 12] = 0;
        jacobians[1][10 + 12] = -cost_function_weight_ * alpha_j_k;
        jacobians[1][11 + 12] = 0;

        jacobians[1][0 + 12 * 2] = 0;
        jacobians[1][1 + 12 * 2] = 0;
        jacobians[1][2 + 12 * 2] = 0;
        jacobians[1][3 + 12 * 2] = 0;
        jacobians[1][4 + 12 * 2] = 0;
        jacobians[1][5 + 12 * 2] = 0;
        jacobians[1][6 + 12 * 2] = 0;
        jacobians[1][7 + 12 * 2] = 0;
        jacobians[1][8 + 12 * 2] = 0;
        jacobians[1][9 + 12 * 2] = 0;
        jacobians[1][10 + 12 * 2] = 0;
        jacobians[1][11 + 12 * 2] = -cost_function_weight_ * alpha_j_k;
      }
    }
    return true;
  }

 private:
  Eigen::Vector3d g_j_;
  Eigen::Vector3d g_k_;
  double cost_function_weight_;
};

class ConCostFunction : public ceres::CostFunction {
 public:
  // constructor
  ConCostFunction(double cost_function_weight,
                  std::vector<Eigen::Vector3d> vector_g, Eigen::Vector3d source,
                  Eigen::Vector3d target) {
    // define residual and block_size
    source_ = source;
    target_ = target;
    vector_g_ = vector_g;
    cost_function_weight_ = sqrt(cost_function_weight);

    nodes_connectivity = vector_g_.size() - 1;

    // set the size of the parameters input
    set_num_residuals(3);
    std::vector<int>* block_sizes = mutable_parameter_block_sizes();
    for (int i = 0; i < nodes_connectivity; ++i) block_sizes->push_back(12);

    // equation (3)
    w_j_.clear();
    for (int i = 0; i < nodes_connectivity; ++i)
      w_j_.push_back(pow(1 - (source_ - vector_g_[i]).squaredNorm() /
                                 (source_ - vector_g_.back()).squaredNorm(),
                         2));

    double normalization_factor = 0;
    for (int i = 0; i < nodes_connectivity; ++i)
      normalization_factor += w_j_[i];

    for (int i = 0; i < nodes_connectivity; ++i)
      w_j_[i] /= normalization_factor;
  }

  ~ConCostFunction() {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    Eigen::Map<Eigen::MatrixXd> res(residuals, 1, 3);

    // back to equation (8)
    Eigen::Vector3d new_node_position;
    new_node_position << 0, 0, 0;
    for (int i = 0; i < nodes_connectivity; ++i) {
      Eigen::Map<const Eigen::Matrix3d> R_j(parameters[i]);
      Eigen::Map<const Eigen::Vector3d> t_j(parameters[i] + 9);
      new_node_position +=
          w_j_[i] * (R_j * (source_ - vector_g_[i]) + vector_g_[i] + t_j);
    }

    res(0, 0) = cost_function_weight_ * (new_node_position - target_)(0);
    res(0, 1) = cost_function_weight_ * (new_node_position - target_)(1);
    res(0, 2) = cost_function_weight_ * (new_node_position - target_)(2);

    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        for (int i = 0; i < w_j_.size(); ++i) {
          jacobians[i][0] =
              cost_function_weight_ * w_j_[i] * (source_(0) - vector_g_[i](0));
          jacobians[i][1] = 0;
          jacobians[i][2] = 0;
          jacobians[i][3] =
              cost_function_weight_ * w_j_[i] * (source_(1) - vector_g_[i](1));
          jacobians[i][4] = 0;
          jacobians[i][5] = 0;
          jacobians[i][6] =
              cost_function_weight_ * w_j_[i] * (source_(2) - vector_g_[i](2));
          jacobians[i][7] = 0;
          jacobians[i][8] = 0;
          jacobians[i][9] = cost_function_weight_ * w_j_[i];
          jacobians[i][10] = 0;
          jacobians[i][11] = 0;

          jacobians[i][0 + 12] = 0;
          jacobians[i][1 + 12] =
              cost_function_weight_ * w_j_[i] * (source_(0) - vector_g_[i](0));
          jacobians[i][2 + 12] = 0;
          jacobians[i][3 + 12] = 0;
          jacobians[i][4 + 12] =
              cost_function_weight_ * w_j_[i] * (source_(1) - vector_g_[i](1));
          jacobians[i][5 + 12] = 0;
          jacobians[i][6 + 12] = 0;
          jacobians[i][7 + 12] =
              cost_function_weight_ * w_j_[i] * (source_(2) - vector_g_[i](2));
          jacobians[i][8 + 12] = 0;
          jacobians[i][9 + 12] = 0;
          jacobians[i][10 + 12] = cost_function_weight_ * w_j_[i];
          jacobians[i][11 + 12] = 0;

          jacobians[i][0 + 12 * 2] = 0;
          jacobians[i][1 + 12 * 2] = 0;
          jacobians[i][2 + 12 * 2] =
              cost_function_weight_ * w_j_[i] * (source_(0) - vector_g_[i](0));
          jacobians[i][3 + 12 * 2] = 0;
          jacobians[i][4 + 12 * 2] = 0;
          jacobians[i][5 + 12 * 2] =
              cost_function_weight_ * w_j_[i] * (source_(1) - vector_g_[i](1));
          jacobians[i][6 + 12 * 2] = 0;
          jacobians[i][7 + 12 * 2] = 0;
          jacobians[i][8 + 12 * 2] =
              cost_function_weight_ * w_j_[i] * (source_(2) - vector_g_[i](2));
          jacobians[i][9 + 12 * 2] = 0;
          jacobians[i][10 + 12 * 2] = 0;
          jacobians[i][11 + 12 * 2] = cost_function_weight_ * w_j_[i];
        }
      }
    }
    return true;
  }

 private:
  Eigen::Vector3d source_;
  Eigen::Vector3d target_;
  std::vector<Eigen::Vector3d> vector_g_;
  std::vector<double> w_j_;
  int nodes_connectivity;
  double cost_function_weight_;
};

}  // namespace core
}  // namespace aid4crop
