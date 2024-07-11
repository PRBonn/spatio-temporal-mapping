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
#ifndef READ_GRAPH_OBJ_HPP
#define READ_GRAPH_OBJ_HPP

#include <Eigen/Core>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace libgraphcpp {
inline bool readGraphOBJ(const std::string obj_file_name,
                         Eigen::MatrixXd& V_out, Eigen::MatrixXi& E_out) {
  bool verbose = false;

  std::vector<std::vector<double> > V;
  std::vector<std::vector<int> > E;

  // Open file, and check for error
  FILE* obj_file = fopen(obj_file_name.c_str(), "r");
  if (NULL == obj_file) {
    fprintf(stderr, "IOError: %s could not be opened...\n",
            obj_file_name.c_str());
    return false;
  }

  V.clear();
  E.clear();

  // variables and constants to assist parsing the .obj file
  // Constant strings to compare against
  std::string v("v");
  std::string e("l");
  std::string tic_tac_toe("#");

#ifndef IGL_LINE_MAX
#define IGL_LINE_MAX 2048
#endif

  char line[IGL_LINE_MAX];
  int line_no = 1;
  while (fgets(line, IGL_LINE_MAX, obj_file) != NULL) {
    char type[IGL_LINE_MAX];
    // Read first word containing type
    if (sscanf(line, "%s", type) == 1) {
      // Get pointer to rest of line right after type
      char* l = &line[strlen(type)];
      if (type == v) {
        std::istringstream ls(&line[1]);
        std::vector<double> vertex{std::istream_iterator<double>(ls),
                                   std::istream_iterator<double>()};

        if (vertex.size() < 3) {
          fprintf(stderr,
                  "Error: readOBJ() vertex on line %d should have at least 3 "
                  "coordinates",
                  line_no);
          fclose(obj_file);
          return false;
        }

        V.push_back(vertex);
      } else if (type == e) {
        std::istringstream ls(&line[1]);
        std::vector<int> edge{std::istream_iterator<int>(ls),
                              std::istream_iterator<int>()};

        if (edge.size() != 2) {
          fprintf(
              stderr,
              "Error: readOBJ() vertex on line %d should have 2 coordinates",
              line_no);
          fclose(obj_file);
          return false;
        }

        E.push_back(edge);
      } else if (strlen(type) >= 1 &&
                 (type[0] == '#' || type[0] == 'g' || type[0] == 's' ||
                  strcmp("usemtl", type) == 0 || strcmp("mtllib", type) == 0)) {
        // ignore comments or other shit
      } else {
        // ignore any other lines
        if (verbose)
          fprintf(stderr,
                  "Warning: readOBJ() ignored non-comment line %d:\n  %s",
                  line_no, line);
      }
    } else {
      // ignore empty line
    }
    line_no++;
  }
  fclose(obj_file);

  int num_vertices = V.size();
  int num_edges = E.size();
  V_out.resize(num_vertices, 3);
  E_out.resize(num_edges, 2);

  for (int i = 0; i < num_vertices; i++) {
    if (V[i].size() != 3) {
      std::cout << "Error in readGraphOBJ.hpp: wrong input dimension\n";
      exit(0);
    }

    for (int j = 0; j < 3; j++) V_out(i, j) = V[i][j];
  }

  for (int i = 0; i < num_edges; i++) {
    if (E[i].size() != 2) {
      std::cout << "Error in readGraphOBJ.hpp: wrong input dimension\n";
      exit(0);
    }

    for (int j = 0; j < 2; j++) E_out(i, j) = E[i][j];
  }

  // remove one for 0 index references
  E_out = E_out - Eigen::MatrixXi::Constant(E_out.rows(), 2, 1);

  return true;
};
}  // namespace libgraphcpp

#endif
