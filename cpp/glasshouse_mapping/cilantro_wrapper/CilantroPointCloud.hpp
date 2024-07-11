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

#include <cilantro/core/data_containers.hpp>
#include <cilantro/utilities/point_cloud.hpp>
#include <cstddef>
#include <eigen3/Eigen/Dense>

namespace aid4crop {
namespace cilantro_wrapper {

class CilantroPointCloud {
 public:
  CilantroPointCloud(const std::vector<Eigen::Vector3d>&);

  void estimateNormals();
  void computeDeformationField(const float = 0.025f);
  std::vector<Eigen::Transform<double, 3, Eigen::Affine>> non_rigid_icp(
      const CilantroPointCloud&, const float = 0.04f, const float = 3.0f);

  size_t size() const;

 private:
  cilantro::PointCloud3d _pcd;
  float _control_points_resolution;
  cilantro::VectorSet<double, 3> _control_points;
  cilantro::NeighborhoodSet<double> _control_points_nn;
  cilantro::NeighborhoodSet<double> _regularization_nn;
};

}  // namespace cilantro_wrapper
}  // namespace aid4crop
