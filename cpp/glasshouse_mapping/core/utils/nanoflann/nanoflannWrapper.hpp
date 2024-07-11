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
#ifndef NANOFLANN_WRAPPER
#define NANOFLANN_WRAPPER

#include <eigen3/Eigen/Core>
#include <nanoflann.hpp>

class nanoflann_wrapper {
 private:
  Eigen::MatrixXd target_;
  std::shared_ptr<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd>>
      kd_tree_index;

 public:
  nanoflann_wrapper() { this->kd_tree_index = nullptr; }

  nanoflann_wrapper(Eigen::MatrixXd target) : target_(target) {
    // set up kdtree
    int leaf_size = 10;
    int dimensionality = 3;

    this->kd_tree_index =
        std::make_shared<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd>>(
            dimensionality, target_, leaf_size);
    this->kd_tree_index->index->buildIndex();
  }

  ~nanoflann_wrapper() {}

  void add_geometry(Eigen::MatrixXd target) {
    // set up kdtree
    this->target_ = target;
    int leaf_size = 10;
    int dimensionality = 3;

    this->kd_tree_index =
        std::make_shared<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd>>(
            dimensionality, target_, leaf_size);
    this->kd_tree_index->index->buildIndex();
  }

  std::vector<int> return_k_closest_points(Eigen::Vector3d query_point,
                                           int k) const {
    // Query point:
    std::vector<double> query_pt;
    for (int d = 0; d < 3; d++) query_pt.push_back(query_point(d));

    // set wtf vectors
    std::vector<size_t> ret_indexes(k);
    std::vector<double> out_dists_sqr(k);
    nanoflann::KNNResultSet<double> resultSet(k);
    resultSet.init(&ret_indexes.at(0), &out_dists_sqr.at(0));

    // knn search
    this->kd_tree_index->index->findNeighbors(resultSet, &query_pt.at(0),
                                              nanoflann::SearchParams(k));

    // pack result into std::vector<int>
    std::vector<int> indexes;
    for (int i = 0; i < k; i++) {
      indexes.push_back(ret_indexes.at(i));
    }

    return indexes;
  }

  Eigen::VectorXd get_element(const int& idx) { return this->target_.row(idx); }

  std::vector<std::tuple<int, double>> return_k_closest_points_with_distances(
      Eigen::Vector3d query_point, int k) const {
    // Query point:
    std::vector<double> query_pt;
    for (int d = 0; d < 3; d++) query_pt.push_back(query_point(d));

    // set wtf vectors
    std::vector<size_t> ret_indexes(k);
    std::vector<double> out_dists_sqr(k);
    nanoflann::KNNResultSet<double> resultSet(k);
    resultSet.init(&ret_indexes.at(0), &out_dists_sqr.at(0));

    // knn search
    this->kd_tree_index->index->findNeighbors(resultSet, &query_pt.at(0),
                                              nanoflann::SearchParams(k));

    // pack result into std::vector<int>
    std::vector<std::tuple<int, double>> indexes_and_distances;
    for (int i = 0; i < k; i++) {
      indexes_and_distances.push_back({ret_indexes.at(i), out_dists_sqr.at(i)});
    }

    return indexes_and_distances;
  }

  std::vector<int> return_k_closest_points_radius(Eigen::Vector3d query_point,
                                                  int k, double radius) const {
    // Query point:
    std::vector<double> query_pt;
    for (int d = 0; d < 3; d++) query_pt.push_back(query_point(d));

    // set wtf vectors
    std::vector<size_t> ret_indexes(k);
    std::vector<double> out_dists_sqr(k);
    nanoflann::KNNResultSet<double> resultSet(k);
    resultSet.init(&ret_indexes.at(0), &out_dists_sqr.at(0));

    // radius search
    this->kd_tree_index->index->findNeighbors(resultSet, &query_pt.at(0),
                                              nanoflann::SearchParams(k));

    // pack result into std::vector<int>
    std::vector<int> indexes;
    for (int i = 0; i < k; i++) {
      if (out_dists_sqr.at(i) < radius) {
        indexes.push_back(ret_indexes.at(i));
      }
    }

    return indexes;
  }
};

#endif
