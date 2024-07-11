// ----------------------------------------------------------------------------
// NOTE: This file has been adapted from the following project, but copyright
// still belongs to them. All rights reserved
// ----------------------------------------------------------------------------
// -                https://github.com/rFalque/libGraphCpp                    -
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
#ifndef POLYSCOPE_WRAPPER_HPP
#define POLYSCOPE_WRAPPER_HPP

#include <Eigen/Core>
#include <string>

#include "polyscope/curve_network.h"
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/view.h"

namespace visualization {

inline void init() {
  polyscope::options::autocenterStructures = true;
  polyscope::view::windowWidth = 1024;
  polyscope::view::windowHeight = 1024;
  polyscope::view::style = polyscope::view::NavigateStyle::Free;
}

inline void close() { polyscope::removeAllStructures(); }

// meshes
inline void add_mesh(const Eigen::MatrixXd& vertices,
                     const Eigen::MatrixXi& faces, std::string mesh_name) {
  polyscope::registerSurfaceMesh(mesh_name, vertices, faces);
  polyscope::getSurfaceMesh(mesh_name)->setSurfaceColor(glm::vec3{0.1, 0.1, 1});
  polyscope::view::resetCameraToHomeView();
}

inline void add_color_to_mesh(const Eigen::MatrixXd& colors,
                              std::string mesh_name, std::string color_name) {
  if (colors.rows() != 0) {
    polyscope::getSurfaceMesh(mesh_name)->addVertexColorQuantity(color_name,
                                                                 colors);
    polyscope::getSurfaceMesh(mesh_name)
        ->getQuantity(color_name)
        ->setEnabled(true);
  }
}

// cloud
inline void add_cloud(const Eigen::MatrixXd& cloud, std::string cloud_name) {
  polyscope::registerPointCloud(cloud_name, cloud);
  polyscope::getPointCloud(cloud_name)->setPointColor(glm::vec3{0.1, 0.1, 1});
  polyscope::view::resetCameraToHomeView();
}

inline void add_color_to_cloud(const Eigen::MatrixXd& colors,
                               std::string cloud_name, std::string color_name) {
  if (colors.rows() != 0) {
    polyscope::getPointCloud(cloud_name)->addColorQuantity(color_name, colors);
    polyscope::getPointCloud(cloud_name)
        ->getQuantity(color_name)
        ->setEnabled(true);
  }
}

// graph
inline void add_graph(const Eigen::MatrixXd& nodes,
                      const Eigen::MatrixXi& edges, std::string graph_name) {
  polyscope::registerPointCloud(graph_name + "_nodes", nodes);
  polyscope::getPointCloud(graph_name + "_nodes")
      ->setPointColor(glm::vec3{1, 0, 0});
  polyscope::registerCurveNetwork(graph_name + "_edges", nodes, edges);
  polyscope::getCurveNetwork(graph_name + "_edges")
      ->setColor(glm::vec3{0, 0, 0});
}

inline void add_color_to_graph(const Eigen::MatrixXd& nodes_colors,
                               const Eigen::MatrixXd& edges_colors,
                               std::string graph_name, std::string color_name) {
  if (nodes_colors.rows() != 0) {
    polyscope::getPointCloud(graph_name + "_nodes")
        ->addColorQuantity(color_name, nodes_colors);
    polyscope::getPointCloud(graph_name + "_nodes")
        ->getQuantity(color_name)
        ->setEnabled(true);
  }
  if (edges_colors.rows() != 0) {
    polyscope::getCurveNetwork(graph_name + "_edges")
        ->addEdgeColorQuantity(color_name, edges_colors);
    polyscope::getCurveNetwork(graph_name + "_edges")
        ->getQuantity(color_name)
        ->setEnabled(true);
  }
}

inline void add_vector_quantity_to_graph(const Eigen::VectorXd& nodes_colors,
                                         const Eigen::VectorXd& edges_colors,
                                         std::string graph_name,
                                         std::string vector_quantity_name) {
  if (nodes_colors.rows() != 0) {
    polyscope::getPointCloud(graph_name + "_nodes")
        ->addVectorQuantity(vector_quantity_name, nodes_colors);
    polyscope::getPointCloud(graph_name + "_nodes")
        ->getQuantity(vector_quantity_name)
        ->setEnabled(true);
  }
  if (edges_colors.rows() != 0) {
    polyscope::getCurveNetwork(graph_name + "_edges")
        ->addEdgeVectorQuantity(vector_quantity_name, edges_colors);
    polyscope::getCurveNetwork(graph_name + "_edges")
        ->getQuantity(vector_quantity_name)
        ->setEnabled(true);
  }
}

inline void add_scalar_quantity_to_graph(const Eigen::VectorXd& nodes_colors,
                                         const Eigen::VectorXd& edges_colors,
                                         std::string graph_name,
                                         std::string scalar_quantity_name) {
  if (nodes_colors.rows() != 0) {
    polyscope::getPointCloud(graph_name + "_nodes")
        ->addScalarQuantity(scalar_quantity_name, nodes_colors);
    polyscope::getPointCloud(graph_name + "_nodes")
        ->getQuantity(scalar_quantity_name)
        ->setEnabled(true);
  }
  if (edges_colors.rows() != 0) {
    polyscope::getCurveNetwork(graph_name + "_edges")
        ->addEdgeScalarQuantity(scalar_quantity_name, edges_colors);
    polyscope::getCurveNetwork(graph_name + "_edges")
        ->getQuantity(scalar_quantity_name)
        ->setEnabled(true);
  }
}

// vectors
inline void add_vectors(const Eigen::MatrixXd& vectors_begin,
                        const Eigen::MatrixXd& vectors_end,
                        std::string vectors_name) {
  polyscope::registerPointCloud(vectors_name, vectors_begin);
  polyscope::getPointCloud(vectors_name)
      ->addVectorQuantity("vectors", vectors_end - vectors_begin,
                          polyscope::VectorType::STANDARD);
  polyscope::getPointCloud(vectors_name)
      ->getQuantity("vectors")
      ->setEnabled(true);
  polyscope::view::resetCameraToHomeView();
}

// tools
inline void show() { polyscope::show(); }

inline void screenshot(std::string screenshot_path) {
  polyscope::screenshot(screenshot_path, false);
}

inline void clear_structures() { polyscope::removeAllStructures(); }

};  // namespace visualization

#endif
