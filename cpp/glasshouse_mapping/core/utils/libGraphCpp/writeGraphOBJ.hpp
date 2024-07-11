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
#ifndef WRITE_GRAPH_OBJ_HPP
#define WRITE_GRAPH_OBJ_HPP

#include <Eigen/Core>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace libgraphcpp {
inline bool writeGraphOBJ(const Eigen::MatrixXd& V_in,
                          const Eigen::MatrixXi& E_in,
                          const std::string obj_file_name) {
  bool verbose = false;

  using namespace std;
  using namespace Eigen;
  assert(V_in.cols() == 3 && "V should have 3 columns");
  ofstream s(obj_file_name);
  if (!s.is_open()) {
    fprintf(stderr, "IOError: writeOBJ() could not open %s\n",
            obj_file_name.c_str());
    return false;
  }
  s << V_in.format(IOFormat(FullPrecision, DontAlignCols, " ", "\n", "v ", "",
                            "", "\n"))
    << (E_in.array() + 1)
           .format(IOFormat(FullPrecision, DontAlignCols, " ", "\n", "l ", "",
                            "", "\n"));
  return true;
};
}  // namespace libgraphcpp

#endif
