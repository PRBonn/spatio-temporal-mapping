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
#include "DeformationGraph.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <ceres/types.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/visualization/utility/DrawGeometry.h>
#include <yaml-cpp/node/detail/node.h>

#include <execution>
#include <fstream>
#include <memory>
#include <numeric>
#include <ranges>
#include <sstream>
#include <string>

#include "VoxelizedPointCloud.hpp"
#include "utils/libGraphCpp/graph.hpp"
#include "utils/nanoflann/nanoflannWrapper.hpp"

namespace aid4crop {
namespace core {

DeformationGraph::DeformationGraph(const VoxelizedPointCloud& source,
                                   const float& resolution,
                                   const int nodes_connectivity)
    : _resolution(resolution),
      _nodes_connectivity(nodes_connectivity),
      _nodes_kdtree(nanoflann_wrapper()),
      _cloud_voxel_size(source.get_voxel_size()) {
  // Initialization
  const float nn_radius = 1.5 * resolution;

  // Get the point cloud to deform and compute the kdtree
  this->_cloud = source.get_cloud();
  this->_cloud_kdtree.add_geometry(this->_cloud);

  // Compute the downsampled version of the pcd (nodes of the graph)
  std::tie(this->_nodes, this->_node_to_cloud_ids) =
      this->downsampling(this->_cloud, resolution);

  // Compute the edges
  this->_edges =
      Eigen::MatrixXi(this->_nodes.rows() * (this->_nodes_connectivity + 1), 2);
  this->_nodes_kdtree.add_geometry(this->_nodes);
  int counter = 0;
  for (int i = 0; i < this->_nodes.rows(); ++i) {
    std::vector<int> closest_points;
    closest_points = this->_nodes_kdtree.return_k_closest_points(
        this->_nodes.row(i), this->_nodes_connectivity + 2);

    bool found_similar = false;
    for (int j = 0; j < closest_points.size(); ++j)
      if (i != closest_points[j]) {
        this->_edges(counter, 0) = i;
        this->_edges(counter, 1) = closest_points[j];
        counter++;
      } else {
        found_similar = true;
      }
  }

  this->_deformation_graph_ptr =
      std::make_shared<libgraphcpp::Graph>(this->_nodes, this->_edges);
}

// target_points: list of points (same of the deformation graph) but in the
// position that we want them after the deformation
Eigen::MatrixXd DeformationGraph::deform(
    const std::vector<Eigen::Vector3d>& source_points,
    const std::vector<Eigen::Vector3d>& target_points, const double min_x,
    const double max_x) {
  // Initialization
  double radius = 0.005f * 3;
  double w_rot = 1.0;
  double w_reg = 10.0;
  double w_con = 10.0;

  // Take references to only nodes inside the given limits (min_x, max_x)
  std::vector<int> nodesIds_to_consider;
  std::vector<int> nodeId2validId(this->size(), -1);
  for (int i = 0; i < this->_deformation_graph_ptr->num_nodes(); ++i) {
    const auto& current_node_x = this->_deformation_graph_ptr->get_node(i)[0];
    if (current_node_x >= min_x and current_node_x <= max_x) {
      nodesIds_to_consider.emplace_back(i);
      nodeId2validId[i] = nodesIds_to_consider.size() - 1;
    }
  }

  // Pre-compute informations about the source points
  std::vector<Eigen::Vector3d> valid_source_points, valid_target_points;
  std::vector<std::vector<int>> sources_nodes_neighbours;
  valid_source_points.reserve(source_points.size());
  valid_target_points.reserve(target_points.size());
  sources_nodes_neighbours.reserve(source_points.size());
  for (int i = 0; i < source_points.size(); ++i) {
    // Ignore points outside the limits
    if (source_points[i][0] < min_x || source_points[i][0] > max_x) continue;

    // Find closest point on the undeformed cloud (the kdtree is pre-computed on
    // the undeformed cloud)
    std::vector<int> closest_point;
    closest_point = this->_cloud_kdtree.return_k_closest_points_radius(
        source_points[i], 1, radius);

    // Ignore this source point if it has no nearest point in the cloud
    if (closest_point.size() <= 0) continue;

    // Take the nodes near to this source point
    auto nearest_nodes = this->_nodes_kdtree.return_k_closest_points(
        this->_cloud.row(closest_point.at(0)), this->_nodes_connectivity + 1);

    // Take only the nearest nodes inside the limits
    std::vector<int> valid_nearest_nodes;
    for (const auto& id : nearest_nodes) {
      if (nodeId2validId[id] != -1) {
        valid_nearest_nodes.emplace_back(id);
      }
    }

    // Ignore this source point if we don't have any valid nearest node
    if (valid_nearest_nodes.size() <= 0) continue;

    // Save all the information about this valid source point
    valid_source_points.emplace_back(source_points[i]);
    valid_target_points.emplace_back(target_points[i]);
    sources_nodes_neighbours.emplace_back(valid_nearest_nodes);
  }

  // Initialize the parameters that need to be optimized (nodes rotations and
  // translations) (only valid nodes considered)
  std::vector<std::vector<double>> params;
  params.resize(nodesIds_to_consider.size());
  for (int i = 0; i < params.size(); ++i) {
    params[i].resize(12);

    // R
    params[i][0] = 1;
    params[i][1] = 0;
    params[i][2] = 0;
    params[i][3] = 0;
    params[i][4] = 1;
    params[i][5] = 0;
    params[i][6] = 0;
    params[i][7] = 0;
    params[i][8] = 1;

    // t
    params[i][9] = 0;
    params[i][10] = 0;
    params[i][11] = 0;
  }

  // Initialize the ceres optimizer
  ceres::Problem problem;

  // Add the residuals to constrain the rotation matrices to be rotation
  // matrices: the three columns must be unit length and all columns must be
  // orthogonal to each other
  for (int i = 0; i < nodesIds_to_consider.size(); ++i) {
    RotCostFunction* cost_function = new RotCostFunction(w_rot);
    problem.AddResidualBlock(cost_function, NULL, &params[i][0]);
  }

  // Add the residuals to constrain consistency between transformations of near
  // nodes
  for (int i = 0; i < nodesIds_to_consider.size(); ++i) {
    for (int j = 0; j < this->_deformation_graph_ptr
                            ->get_adjacency_list(nodesIds_to_consider[i])
                            .size();
         ++j) {
      int k = this->_deformation_graph_ptr->get_adjacency_list(
          nodesIds_to_consider[i], j);

      // Ignore this "connection" if the neighbor is outside the limit
      if (nodeId2validId[k] == -1) continue;

      // Get the corresponding nodes
      Eigen::Vector3d g_j =
          this->_deformation_graph_ptr->get_node(nodesIds_to_consider[i]);
      Eigen::Vector3d g_k = this->_deformation_graph_ptr->get_node(k);

      RegCostFunction* cost_function = new RegCostFunction(w_reg, g_j, g_k);

      // add residual block
      problem.AddResidualBlock(cost_function, NULL, &params[i][0],
                               &params[nodeId2validId[k]][0]);
    }
  }

  // Add the residuals to constrain nodes for which we have associations to
  // move where we want them to move
  for (int i = 0; i < valid_source_points.size(); ++i) {
    // stack the parameters
    std::vector<double*> parameter_blocks;
    std::vector<Eigen::Vector3d> vector_g;

    for (int j = 0; j < sources_nodes_neighbours[i].size() - 1; ++j) {
      parameter_blocks.push_back(
          &params[nodeId2validId[sources_nodes_neighbours[i][j]]][0]);
    }
    for (int j = 0; j < sources_nodes_neighbours[i].size(); ++j) {
      vector_g.push_back(this->_deformation_graph_ptr->get_node(
          sources_nodes_neighbours[i][j]));
    }

    // create the cost function
    ConCostFunction* cost_function = new ConCostFunction(
        w_con, vector_g, valid_source_points[i], valid_target_points[i]);

    // add residual block
    problem.AddResidualBlock(cost_function, NULL, parameter_blocks);
  }

  // Run the solver
  ceres::Solver::Options options;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.check_gradients = false;
  options.gradient_check_relative_precision = 0.01;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 4;
  options.max_num_iterations = 100;
  options.function_tolerance = 1e-10;
  options.parameter_tolerance = 1e-10;
  options.dense_linear_algebra_library_type = ceres::LAPACK;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  // VERBOSE
  /* std::cout << summary.FullReport() << std::endl; */

  // If the optimization failed exit without computing the deformation
  if (summary.termination_type == 2) {
    return this->_cloud;
  }

  // Extract the optimized parameters
  std::vector<Eigen::Matrix3d> rotation_matrices;
  std::vector<Eigen::Vector3d> translation_vectors;
  rotation_matrices.resize(this->size());
  translation_vectors.resize(this->size());
  for (int i = 0; i < this->size(); ++i) {  // Set everything to zero
    rotation_matrices[i] = Eigen::Matrix3d::Identity();
    translation_vectors[i] = Eigen::Vector3d::Zero();
  }

  // Update only the optimized nodes
  for (int i = 0; i < nodesIds_to_consider.size(); ++i) {
    rotation_matrices[nodesIds_to_consider[i]] =
        Eigen::Map<Eigen::Matrix3d>(&(params[i][0]));
    translation_vectors[nodesIds_to_consider[i]] =
        Eigen::Map<Eigen::Vector3d>(&(params[i][0]) + 9);
  }

  // Deform all points in the original cloud
  auto deformed_cloud = this->_cloud;

  std::transform(
      std::execution::par, this->_cloud.rowwise().cbegin(),
      this->_cloud.rowwise().cend(), deformed_cloud.rowwise().begin(),
      [&, this](const Eigen::Vector3d& current_point) {
        // Get the node neighbors of the current point
        std::vector<int> neighbours_nodes =
            this->_nodes_kdtree.return_k_closest_points(
                current_point, this->_nodes_connectivity + 1);
        const int neighbours_node_back = neighbours_nodes.back();
        neighbours_nodes.pop_back();

        // Initialize some utility function for next procedures
        const auto compute_distance = [&, this](const auto& node_idx) {
          return (current_point -
                  this->_deformation_graph_ptr->get_node(node_idx))
              .squaredNorm();
        };
        const double denominator = compute_distance(neighbours_node_back);

        // Compute the weight from each neighbor node of the current point
        std::vector<double> w_j(_nodes_connectivity);
        std::transform(neighbours_nodes.cbegin(), neighbours_nodes.cend(),
                       w_j.begin(), [&](const auto& nn_idx) {
                         return std::pow(
                             1.0 - compute_distance(nn_idx) / denominator, 2.0);
                       });
        const double normalization_factor =
            std::reduce(w_j.cbegin(), w_j.cend(), 0.0);

        Eigen::Vector3d new_point = Eigen::Vector3d().setZero();
        for (int j = 0; j < _nodes_connectivity; ++j) {
          const auto& nn_idx = neighbours_nodes[j];
          const auto& node = _deformation_graph_ptr->get_node(nn_idx);
          new_point += (w_j[j] / normalization_factor) *
                       (rotation_matrices[nn_idx] * (current_point - node) +
                        node + translation_vectors[nn_idx]);
        }

        return new_point;
      });

  return deformed_cloud;
}

size_t DeformationGraph::size() const { return this->_nodes.rows(); }

Eigen::MatrixXd DeformationGraph::get_cloud() const { return this->_cloud; }

void DeformationGraph::visualize() const {
  const auto graph_nodes = this->_deformation_graph_ptr->get_nodes();
  const auto graph_edges = this->_deformation_graph_ptr->get_edges();

  // Create a open3d version of the nodes points
  auto new_pcd = open3d::geometry::PointCloud();
  for (int i = 0; i < graph_nodes.rows(); ++i) {
    new_pcd.points_.emplace_back(graph_nodes.row(i));
    new_pcd.colors_.emplace_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  }

  // Create a line for each edge
  std::vector<Eigen::Vector2i> lines;
  for (int r = 0; r < graph_edges.rows(); ++r) {
    lines.emplace_back(Eigen::Vector2i(graph_edges.row(r)));
  }
  auto lineSet_ptr =
      std::make_shared<open3d::geometry::LineSet>(new_pcd.points_, lines);

  // Visualize the pcd + the edge lines
  auto nodes_ptr = std::make_shared<open3d::geometry::PointCloud>(new_pcd);
  open3d::visualization::DrawGeometries({nodes_ptr, lineSet_ptr});
}

void DeformationGraph::visualize(const DeformationGraph& other) const {
  const auto graph_nodes = this->_deformation_graph_ptr->get_nodes();
  const auto other_graph_nodes = other._deformation_graph_ptr->get_nodes();
  const auto graph_edges = this->_deformation_graph_ptr->get_edges();
  const auto other_graph_edges = other._deformation_graph_ptr->get_edges();

  // Create a open3d version of the nodes points
  auto new_pcd = open3d::geometry::PointCloud();
  for (int i = 0; i < graph_nodes.rows(); ++i) {
    new_pcd.points_.emplace_back(graph_nodes.row(i));
    new_pcd.colors_.emplace_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  }
  auto other_new_pcd = open3d::geometry::PointCloud();
  for (int i = 0; i < other_graph_nodes.rows(); ++i) {
    other_new_pcd.points_.emplace_back(other_graph_nodes.row(i));
    other_new_pcd.colors_.emplace_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  }

  // Create a line for each edge
  std::vector<Eigen::Vector2i> lines;
  std::vector<Eigen::Vector3d> lines_color;
  for (int r = 0; r < graph_edges.rows(); ++r) {
    lines.emplace_back(Eigen::Vector2i(graph_edges.row(r)));
    lines_color.emplace_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  }
  auto lineSet_ptr =
      std::make_shared<open3d::geometry::LineSet>(new_pcd.points_, lines);
  lineSet_ptr->colors_ = lines_color;
  std::vector<Eigen::Vector2i> other_lines;
  std::vector<Eigen::Vector3d> other_lines_color;
  for (int r = 0; r < other_graph_edges.rows(); ++r) {
    other_lines.emplace_back(Eigen::Vector2i(other_graph_edges.row(r)));
    other_lines_color.emplace_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  }
  auto other_lineSet_ptr = std::make_shared<open3d::geometry::LineSet>(
      other_new_pcd.points_, other_lines);
  other_lineSet_ptr->colors_ = other_lines_color;

  // Visualize the pcd + the edge lines
  auto nodes_ptr = std::make_shared<open3d::geometry::PointCloud>(new_pcd);
  auto other_nodes_ptr =
      std::make_shared<open3d::geometry::PointCloud>(other_new_pcd);
  open3d::visualization::DrawGeometries(
      {nodes_ptr, lineSet_ptr, other_nodes_ptr, other_lineSet_ptr});
}

// ########## PRIVATE METHODS ##########
// NO ONE FOR NOW

}  // namespace core
}  // namespace aid4crop
