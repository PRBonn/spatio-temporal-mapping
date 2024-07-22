// MIT License
//
// Copyright (c) 2024 Luca Lobefaro, Meher V.R. Malladi, Tiziano Guadagnino, Cyrill Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
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

#include <Eigen/Core>

namespace Eigen {
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace st_mapping {

class DeformationGraph {
public:
    DeformationGraph(const std::vector<Eigen::Vector3d> &cloud,
                     const double resolution,
                     const int nodes_connectivity,
                     const double graph_consistency_weight,
                     const double deformation_weight);

    using Voxel = Eigen::Vector3i;
    struct VoxelHash {
        size_t operator()(const Voxel &voxel) const {
            const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
            return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
        }
    };

    void AddMeasurement(const Eigen::Vector3d &source_point, const Eigen::Vector3d &target_point);

    void AddMeasurements(const std::vector<Eigen::Vector3d> &source_points,
                         const std::vector<Eigen::Vector3d> &target_points);

    std::vector<Eigen::Vector3d> Deform(const int max_num_iterations);

    std::vector<int> GetNearestNodes(const Eigen::Vector3d &point,
                                     const int max_neighbors,
                                     const bool ignore_first) const;

    inline std::pair<std::vector<Eigen::Vector3d>, std::unordered_map<int, std::vector<int>>>
    GetGraph() const {
        return std::make_pair(_nodes, _edges);
    };

private:
    void _ExtractNodesFromCloud();
    void _ExtractEdges();
    void _InitializeProblemContraints();

    std::vector<Eigen::Vector3d> _cloud;  // TODO: avoid copy
                                          // (an idea is to pass the cloud in Deform() again)
    std::vector<Eigen::Vector3d> _nodes;
    std::unordered_map<Voxel, int, VoxelHash> _nodes_voxels;
    std::unordered_map<int, std::vector<int>> _edges;  // TODO: Is this really needed?
    const double _resolution;
    const int _nodes_connectivity;

    ceres::Problem _problem;
    std::vector<Eigen::Vector6d> _state;

    const double _graph_consistency_weight;
    const double _deformation_weight;
};
}  // namespace st_mapping
