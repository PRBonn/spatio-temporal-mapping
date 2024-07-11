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
#ifndef PLOT_GRAPH_HPP
#define PLOT_GRAPH_HPP
#include <Eigen/Core>
#include <string>

#include "graph.hpp"
#include "graphOptions.hpp"
#include "polyscopeWrapper.hpp"

namespace visualization {

inline void add_graph(libgraphcpp::Graph graph, std::string graph_name) {
  Eigen::MatrixXd nodes = graph.get_nodes();
  Eigen::MatrixXi edges = graph.get_edges();

  visualization::init();
  visualization::add_graph(nodes, edges, graph_name);
};

inline void add_graph(libgraphcpp::Graph graph) {
  visualization::add_graph(graph, "graph");
};

inline void plot(libgraphcpp::Graph graph, std::string graph_name) {
  visualization::add_graph(graph, graph_name);
  visualization::show();
  visualization::clear_structures();
};

inline void plot(libgraphcpp::Graph graph) {
  visualization::plot(graph, "graph");
};

inline void return_colors_highlight(std::vector<int> element_to_highlight,
                                    int size_array, Eigen::MatrixXd& colors) {
  colors = Eigen::MatrixXd::Constant(size_array, 3, 0.1);

  for (int element : element_to_highlight) {
    colors(element, 0) = 0.1;
    colors(element, 1) = 0.1;
    colors(element, 2) = 1.0;
  }
};

inline void plot_connectivity(libgraphcpp::Graph graph) {
  graph.connectivity_tests();
  Eigen::MatrixXd empty;  // used to pass empty content
  Eigen::MatrixXd highlights;

  visualization::init();
  visualization::clear_structures();
  visualization::add_graph(graph, "graph_connectivity");

  // add color for the one cut vertices
  std::vector<int> one_cut_vertices;
  graph.is_biconnected(one_cut_vertices);
  return_colors_highlight(one_cut_vertices, graph.num_nodes(), highlights);
  visualization::add_color_to_graph(highlights, empty, "graph_connectivity",
                                    "one_cut_vertices");

  // add color for the two cut vertices
  std::vector<std::pair<int, int> > two_cut_vertices;
  graph.is_triconnected(two_cut_vertices);
  std::vector<int> temp;
  for (std::pair<int, int> p : two_cut_vertices) {
    temp.push_back(p.first);
    temp.push_back(p.second);
  }
  return_colors_highlight(temp, graph.num_nodes(), highlights);
  visualization::add_color_to_graph(highlights, empty, "graph_connectivity",
                                    "two_cut_vertices");

  // add color for the bridges
  std::vector<int> bridges;
  graph.has_bridges(bridges);
  return_colors_highlight(temp, graph.num_edges(), highlights);
  visualization::add_color_to_graph(empty, highlights, "graph_connectivity",
                                    "bridges");

  visualization::show();
  visualization::clear_structures();
};

inline void plot_and_highlight(libgraphcpp::Graph graph,
                               std::vector<int> node_list,
                               std::vector<int> edge_list) {
  visualization::init();
  visualization::clear_structures();
  visualization::add_graph(graph, "graph");

  Eigen::MatrixXd nodes_colors, edges_colors;

  return_colors_highlight(node_list, graph.num_nodes(), nodes_colors);
  return_colors_highlight(edge_list, graph.num_edges(), edges_colors);

  visualization::add_color_to_graph(nodes_colors, edges_colors, "graph",
                                    "highlight");
  visualization::show();
};

}  // namespace visualization

#endif
