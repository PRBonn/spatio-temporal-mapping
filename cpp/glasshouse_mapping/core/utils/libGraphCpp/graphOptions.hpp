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
#ifndef GRAPH_OPTIONS_HPP
#define GRAPH_OPTIONS_HPP

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>

struct graphOptions {
  // path to the graph to load
  std::string path_graph_obj;

  // general options
  bool visualization = false;
  bool verbose = false;

  // visualization parameters
  std::vector<double> nodes_color = {1.0, 0.1, 0.1};
  std::vector<double> edges_color = {0.1, 0.1, 0.1};

  void print() {
    std::cout << "list of the parameters:" << std::endl;
    std::cout << std::endl;
    std::cout << "*** IO files: ***" << std::endl;
    std::cout << "path_graph_obj: " << path_graph_obj << std::endl;
    std::cout << std::endl;
    std::cout << "*** General parameters ***" << std::endl;
    std::cout << "visualization: " << visualization << std::endl;
    std::cout << "verbose: " << verbose << std::endl;
    std::cout << std::endl;
    std::cout << "*** Visualization parameters ***" << std::endl;
    std::cout << "nodes_color: [" << nodes_color[0] << "," << nodes_color[1]
              << "," << nodes_color[2] << "]" << std::endl;
    std::cout << "edges_color: [" << edges_color[0] << "," << edges_color[1]
              << "," << edges_color[2] << "]" << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
  }

  bool loadYAML(std::string config_file) {
    YAML::Node config = YAML::LoadFile(config_file);

    // IO
    path_graph_obj = config["io_files"]["graph_obj"].as<std::string>();

    // general options
    visualization = config["general_params"]["visualization"].as<bool>();
    verbose = config["general_params"]["verbose"].as<bool>();

    // visualization parameters
    nodes_color =
        config["visualization_params"]["nodes_color"].as<std::vector<double>>();
    edges_color =
        config["visualization_params"]["edges_color"].as<std::vector<double>>();

    if (verbose) print();

    return true;
  }
};

#endif
