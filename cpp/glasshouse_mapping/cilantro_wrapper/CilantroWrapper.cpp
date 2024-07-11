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
#include "CilantroWrapper.hpp"

#include "CilantroPointCloud.hpp"

namespace aid4crop {
namespace cilantro_wrapper {

std::vector<Eigen::Transform<double, 3, Eigen::Affine>> non_rigid_register(
    const std::vector<Eigen::Vector3d>& ref,
    const std::vector<Eigen::Vector3d>& query) {
  // Create the cilantro versions of the two point cloud
  CilantroPointCloud ref_cilantro(ref);
  CilantroPointCloud query_cilantro(query);

  // Pre-computations
  ref_cilantro.estimateNormals();
  ref_cilantro.computeDeformationField();
  query_cilantro.estimateNormals();

  return ref_cilantro.non_rigid_icp(query_cilantro);
}

}  // namespace cilantro_wrapper
}  // namespace aid4crop
