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
#include "CilantroPointCloud.hpp"

#include <cilantro/registration/icp_common_instances.hpp>
#include <iostream>

namespace aid4crop {
namespace cilantro_wrapper {

CilantroPointCloud::CilantroPointCloud(
    const std::vector<Eigen::Vector3d>& points) {
  this->_pcd = cilantro::PointCloud3d(cilantro::ConstDataMatrixMap<double, 3>(
      points[0].data(), 3, points.size()));
}

void CilantroPointCloud::estimateNormals() {
  // Create the KDTree
  cilantro::KDTree3d<> tree(this->_pcd.points);

  // Estimate normals using KDTree
  this->_pcd.estimateNormalsKNN(tree, 7);
  /* this->_pcd.estimateNormalsRadius(tree, 0.01f); */
  /* this->_pcd.estimateNormalsKNNInRadius(tree, 7, 0.05f); */
}

void CilantroPointCloud::computeDeformationField(const float resolution) {
  // Save the control points resolution
  this->_control_points_resolution = resolution;

  // Get a sparse set of control points by downsampling
  this->_control_points =
      cilantro::PointsGridDownsampler3d(this->_pcd.points,
                                        this->_control_points_resolution)
          .getDownsampledPoints();
  cilantro::KDTree<double, 3> control_tree(this->_control_points);

  // Find which control points affect each point
  this->_control_points_nn = control_tree.search(
      this->_pcd.points, cilantro::KNNNeighborhoodSpecification<>(4));

  // Get the regularization neighboorhoods for control nodes
  this->_regularization_nn = control_tree.search(
      this->_control_points, cilantro::KNNNeighborhoodSpecification<>(8));
}

std::vector<Eigen::Transform<double, 3, Eigen::Affine>>
CilantroPointCloud::non_rigid_icp(const CilantroPointCloud& destination,
                                  const float max_correspondence_dist_sq,
                                  const float regularization_sigma_factor) {
  // Initialization
  const float src_to_control_sigma = 0.5f * this->_control_points_resolution;
  const float regularization_sigma =
      regularization_sigma_factor * this->_control_points_resolution;
  cilantro::SimpleCombinedMetricSparseRigidWarpFieldICP3d icp(
      destination._pcd.points, destination._pcd.normals, this->_pcd.points,
      this->_control_points_nn, this->_control_points.cols(),
      this->_regularization_nn);

  // Set ICP parameters
  icp.correspondenceSearchEngine().setMaxDistance(max_correspondence_dist_sq);
  icp.controlWeightEvaluator().setSigma(src_to_control_sigma);
  icp.regularizationWeightEvaluator().setSigma(regularization_sigma);

  icp.setMaxNumberOfIterations(15).setConvergenceTolerance(2.5e-3f);
  icp.setMaxNumberOfGaussNewtonIterations(1).setGaussNewtonConvergenceTolerance(
      5e-4f);
  icp.setMaxNumberOfConjugateGradientIterations(500)
      .setConjugateGradientConvergenceTolerance(1e-5f);
  icp.setPointToPointMetricWeight(0.0f)
      .setPointToPlaneMetricWeight(1.0f)
      .setStiffnessRegularizationWeight(200.0f);
  icp.setHuberLossBoundary(1e-2f);

  // Perform ICP
  auto estimated_transformations = icp.estimate().getDenseWarpField();

  // Convert obtained transformations
  assert(estimated_transformations.size() == this->_pcd.size());
  std::vector<Eigen::Transform<double, 3, Eigen::Affine>> transformations;
  transformations.resize(this->_pcd.size());
  for (int i = 0; i < this->_pcd.size(); ++i) {
    transformations[i] = estimated_transformations[i];
  }

  return transformations;
}

size_t CilantroPointCloud::size() const { return this->_pcd.size(); }

}  // namespace cilantro_wrapper
}  // namespace aid4crop
