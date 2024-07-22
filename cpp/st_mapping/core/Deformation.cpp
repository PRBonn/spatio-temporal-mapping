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
#include "Deformation.hpp"

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>

#include <iostream>
#include <numeric>
#include <set>
#include <sophus/se3.hpp>
#include <vector>

namespace {
static constexpr int StateDim = 6;
static constexpr int MeasurementDim = 3;

struct GraphConsistencyFactor {
    GraphConsistencyFactor(const Eigen::Vector3d &node,
                           const Eigen::Vector3d &neighbor_node,
                           const double weight)
        : _node(node), _neighbor_node(neighbor_node), _weight(sqrt(weight)) {}
    template <typename T>
    bool operator()(const T *const x, const T *const x_neighbor, T *residual) const {
        using StateType = Eigen::Matrix<T, StateDim, 1>;
        using ErrorType = Eigen::Matrix<T, MeasurementDim, 1>;

        // Access to the state poses
        const auto vector_node_pose = Eigen::Map<const StateType>(x);
        const auto node_pose = Sophus::SE3<T>::exp(vector_node_pose);
        const auto vector_neighbor_node_pose = Eigen::Map<const StateType>(x_neighbor);
        const auto neighbor_node_pose = Sophus::SE3<T>::exp(vector_neighbor_node_pose);

        // Compute the error
        auto error = Eigen::Map<ErrorType>(residual);
        error = node_pose.so3() * (_neighbor_node - _node) + _node + node_pose.translation() -
                (_neighbor_node + neighbor_node_pose.translation());
        error *= T(_weight);

        return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d &node,
                                       const Eigen::Vector3d &neighbor_node,
                                       const double weight) {
        return new ceres::AutoDiffCostFunction<GraphConsistencyFactor, MeasurementDim, StateDim,
                                               StateDim>(
            new GraphConsistencyFactor(node, neighbor_node, weight));
    }

private:
    const Eigen::Vector3d _node;
    const Eigen::Vector3d _neighbor_node;
    const double _weight;
};

struct DeformationFactor {
    DeformationFactor(const std::vector<Eigen::Vector3d> &neighbors,
                      const Eigen::Vector3d &source,
                      const Eigen::Vector3d &target,
                      const double weight)
        : _neighbors(neighbors),
          _source(source),
          _target(target),
          _weight(sqrt(weight)),
          _influence_weights(neighbors.size()) {
        // Compute weights
        const double d_max = (source - _neighbors.back()).squaredNorm();
        std::transform(_neighbors.cbegin(), _neighbors.cend(), _influence_weights.begin(),
                       [&](const auto &n) {
                           const double d = (source - n).squaredNorm();
                           const double sqrt_w = 1.0 - d / d_max;
                           return sqrt_w * sqrt_w;
                       });
        // Normalize weights
        const double normalization_factor =
            std::reduce(_influence_weights.cbegin(), _influence_weights.cend(), 0.0);
        std::transform(_influence_weights.cbegin(), _influence_weights.cend(),
                       _influence_weights.begin(),
                       [&](const auto &w) { return w / normalization_factor; });
    }
    template <typename T>
    bool operator()(T const *const *x, T *residual) const {
        using StateType = Eigen::Matrix<T, StateDim, 1>;
        using ErrorType = Eigen::Matrix<T, MeasurementDim, 1>;

        // Compute the prediction
        std::vector<int> neighbors_ids(_neighbors.size());
        std::iota(neighbors_ids.begin(), neighbors_ids.end(), 0);
        ErrorType prediction = std::transform_reduce(
            neighbors_ids.cbegin(), neighbors_ids.cend(), ErrorType().setZero(),
            std::plus<ErrorType>(), [&](const int idx) {
                const auto vector_pose = Eigen::Map<const StateType>(x[idx]);
                const auto pose = Sophus::SE3<T>::exp(vector_pose);
                const auto &neighbor = _neighbors.at(idx);
                return ErrorType(
                    (T(_influence_weights.at(idx)) *
                     (pose.so3() * (_source - neighbor) + neighbor + pose.translation())));
            });

        // Compute the error
        auto error = Eigen::Map<ErrorType>(residual);
        error = T(_weight) * (prediction - _target);

        return true;
    }
    static ceres::DynamicCostFunction *Create(const std::vector<Eigen::Vector3d> &neighbors_,
                                              const Eigen::Vector3d &source_,
                                              const Eigen::Vector3d &target_,
                                              const double weight) {
        auto *cost = new ceres::DynamicAutoDiffCostFunction<DeformationFactor, 4>(
            new DeformationFactor(neighbors_, source_, target_, weight));
        cost->SetNumResiduals(MeasurementDim);
        std::for_each(neighbors_.cbegin(), neighbors_.cend(), [&](const auto &nn) {
            (void)nn;
            cost->AddParameterBlock(StateDim);
        });
        return cost;
    }

private:
    const std::vector<Eigen::Vector3d> _neighbors;
    const Eigen::Vector3d _source;
    const Eigen::Vector3d _target;
    const double _weight;
    std::vector<double> _influence_weights;
};
}  // namespace

namespace st_mapping {
DeformationGraph::DeformationGraph(const std::vector<Eigen::Vector3d> &cloud,
                                   const double resolution,
                                   const int nodes_connectivity,
                                   const double graph_consistency_weight,
                                   const double deformation_weight)
    : _cloud{cloud},
      _resolution(resolution),
      _nodes_connectivity(nodes_connectivity),
      _graph_consistency_weight(graph_consistency_weight),
      _deformation_weight(deformation_weight) {
    this->_ExtractNodesFromCloud();
    this->_ExtractEdges();
    this->_InitializeProblemContraints();
}

void DeformationGraph::_InitializeProblemContraints() {
    // Initialize the state
    _state = std::vector<Eigen::Vector6d>(_nodes.size(), Eigen::Vector6d::Zero());

    // Add edge constraints
    std::for_each(
        _edges.cbegin(), _edges.cend(), [this](const auto &node_id_and_neighbors) mutable {
            const auto &[node_id, neighbors] = node_id_and_neighbors;
            std::for_each(neighbors.cbegin(), neighbors.cend(), [&](const auto &neighbor_id) {
                auto *cost = GraphConsistencyFactor::Create(
                    _nodes.at(node_id), _nodes.at(neighbor_id), _graph_consistency_weight);
                _problem.AddResidualBlock(cost, nullptr, _state.at(node_id).data(),
                                          _state.at(neighbor_id).data());
            });
        });
}

void DeformationGraph::AddMeasurement(const Eigen::Vector3d &source_point,
                                      const Eigen::Vector3d &target_point) {
    // Get nodes in the graph nearest to the source point
    std::vector<int> source_nearest_nodes =
        GetNearestNodes(source_point, _nodes_connectivity + 1, false);

    // Not a valid source point
    if (source_nearest_nodes.size() < 2) return;

    // Create the residual
    std::vector<double *> parameters_block;
    std::vector<Eigen::Vector3d> vector_g;
    parameters_block.reserve(source_nearest_nodes.size());
    vector_g.reserve(source_nearest_nodes.size());
    std::for_each(source_nearest_nodes.cbegin(), source_nearest_nodes.cend(),
                  [&](const int nearest_nodes_idx) {
                      parameters_block.emplace_back(_state.at(nearest_nodes_idx).data());
                      vector_g.emplace_back(_nodes.at(nearest_nodes_idx));
                  });
    auto *conCostFunction =
        DeformationFactor::Create(vector_g, source_point, target_point, _deformation_weight);
    _problem.AddResidualBlock(conCostFunction, nullptr, parameters_block);
}

void DeformationGraph::AddMeasurements(const std::vector<Eigen::Vector3d> &source_points,
                                       const std::vector<Eigen::Vector3d> &target_points) {
    if (source_points.size() != target_points.size()) {
        std::cerr << "Number of source and target points is not the same!" << std::endl;
        exit(1);
    }
    std::vector<int> source_ids(source_points.size());
    std::iota(source_ids.begin(), source_ids.end(), 0);
    std::for_each(source_ids.cbegin(), source_ids.cend(), [&](const int idx) {
        AddMeasurement(source_points.at(idx), target_points.at(idx));
    });
}

std::vector<Eigen::Vector3d> DeformationGraph::Deform(const int max_num_iterations) {
    // Optimize
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = max_num_iterations;
    ceres::Solver::Summary summary;
    options.minimizer_progress_to_stdout = false;
    ceres::Solve(options, &_problem, &summary);

    // Apply defomation according to the optimized transformations
    std::vector<Eigen::Vector3d> deformed_cloud(_cloud.size());
    std::vector<int> cloud_point_ids(_cloud.size());
    std::iota(cloud_point_ids.begin(), cloud_point_ids.end(), 0);
    // TODO: can be parallelized (but check that it actually improves)
    std::transform(
        cloud_point_ids.cbegin(), cloud_point_ids.cend(), deformed_cloud.begin(),
        [&](const int current_point_idx) {
            // Get the node neighbors of the current point
            // TODO: if during the graph building we save which point in the cloud each node is then
            // we can avoid to search for neighbors
            std::vector<int> neighbours_nodes =
                GetNearestNodes(_cloud.at(current_point_idx), _nodes_connectivity + 1, false);
            if (neighbours_nodes.size() < 1) {
                return _cloud.at(current_point_idx);
            }

            // Initialize some utility function for next procedures
            const int neighbours_node_back = neighbours_nodes.back();
            const auto compute_distance = [&, this](const auto &node_idx) {
                return (_cloud.at(current_point_idx) - _nodes.at(node_idx)).squaredNorm();
            };
            const double max_distance = compute_distance(neighbours_node_back);

            // Compute the weight from each neighbor node of the current point
            std::vector<double> w_j(neighbours_nodes.size());
            std::transform(neighbours_nodes.cbegin(), neighbours_nodes.cend(), w_j.begin(),
                           [&](const auto &nn_idx) {
                               return std::pow(1.0 - compute_distance(nn_idx) / max_distance, 2.0);
                           });
            const double normalization_factor = std::reduce(w_j.cbegin(), w_j.cend(), 0.0);

            Eigen::Vector3d new_point = std::transform_reduce(
                neighbours_nodes.cbegin(), neighbours_nodes.cend(), w_j.cbegin(),
                Eigen::Vector3d().setZero(), std::plus<Eigen::Vector3d>{},
                [&](const int nn_idx, const double w) {
                    const Eigen::Vector3d &node = _nodes.at(nn_idx);
                    const Sophus::SE3<double> T = Sophus::SE3<double>::exp(_state.at(nn_idx));
                    return Eigen::Vector3d(
                        (w / normalization_factor) *
                        (T.so3() * (_cloud.at(current_point_idx) - node) + node + T.translation()));
                });

            return new_point;
        });

    return deformed_cloud;
}

std::vector<int> DeformationGraph::GetNearestNodes(const Eigen::Vector3d &point,
                                                   const int max_neighbors,
                                                   const bool ignore_first) const {
    auto distance_ordering = [&](const int idx1, const int idx2) {
        return ((this->_nodes.at(idx1) - point).norm() < (this->_nodes.at(idx2) - point).norm());
    };

    std::set<int, decltype(distance_ordering)> nearest_nodes_ids(distance_ordering);

    const auto voxel = Voxel((point / this->_resolution).template cast<int>());
    for (int i = voxel.x() - 1; i < voxel.x() + 2; ++i) {
        for (int j = voxel.y() - 1; j < voxel.y() + 2; ++j) {
            for (int k = voxel.z() - 1; k < voxel.z() + 2; ++k) {
                auto search = this->_nodes_voxels.find({i, j, k});
                if (search != this->_nodes_voxels.end()) {
                    if (nearest_nodes_ids.size() < static_cast<size_t>(max_neighbors)) {
                        nearest_nodes_ids.insert(search->second);
                    } else if (nearest_nodes_ids.lower_bound(search->second) !=
                               nearest_nodes_ids.end()) {
                        nearest_nodes_ids.erase(*(nearest_nodes_ids.rbegin()));
                        nearest_nodes_ids.insert(search->second);
                    }
                }
            }
        }
    }
    if (nearest_nodes_ids.size() > 0 && ignore_first) {
        nearest_nodes_ids.erase(nearest_nodes_ids.begin());
    }
    std::vector<int> nearest_nodes_ids_vec(nearest_nodes_ids.size());
    std::transform(nearest_nodes_ids.cbegin(), nearest_nodes_ids.cend(),
                   nearest_nodes_ids_vec.begin(), [](const int el) { return el; });
    return nearest_nodes_ids_vec;
}

void DeformationGraph::_ExtractNodesFromCloud() {
    const size_t &cloud_size = this->_cloud.size();
    this->_nodes_voxels.reserve(cloud_size);
    this->_nodes.reserve(cloud_size);

    std::for_each(this->_cloud.cbegin(), this->_cloud.cend(), [&](const auto &point) {
        const auto voxel = Voxel((point / this->_resolution).template cast<int>());
        if (this->_nodes_voxels.find(voxel) == this->_nodes_voxels.end()) {
            this->_nodes_voxels.insert({voxel, this->_nodes.size()});
            this->_nodes.emplace_back(point);
        }
    });

    this->_nodes.shrink_to_fit();
}

void DeformationGraph::_ExtractEdges() {
    std::vector<int> nodes_ids(_nodes.size());
    std::iota(nodes_ids.begin(), nodes_ids.end(), 0);
    _edges.reserve(_nodes.size());
    std::for_each(nodes_ids.cbegin(), nodes_ids.cend(), [&](const int node_idx) {
        _edges.insert({node_idx, GetNearestNodes(_nodes[node_idx], _nodes_connectivity + 1, true)});
    });
}

}  // namespace st_mapping
