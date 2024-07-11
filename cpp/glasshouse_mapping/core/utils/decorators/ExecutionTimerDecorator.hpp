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

#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace aid4crop {
namespace utils {

class ExecutionTimer {
 public:
  // What to do before
  ExecutionTimer(std::string_view);

  // What to do after
  ~ExecutionTimer();

 private:
  std::string_view msg;
  std::chrono::high_resolution_clock::time_point t1;
};

// Usage:
// For normal functions:
//  auto out = compute_execution_time("message here", function_name,
//  function_inputs...);
//
// For class methods (use lambda):
//  auto out= compute_execution_time("message here",
//  [&object_name](lambda_inputs declaration) {return
//  object_name.method_name(lambda_inputs...)}, inputs);
//
// IMPORTANT: explicitly pass default arguments
template <typename FuncT, typename... ArgsT>
auto compute_execution_time(std::string_view msg, FuncT&& func, ArgsT... args) {
  // Create the execution timer
  auto timer = ExecutionTimer(msg);

  // Call function (then the ExecutionTimer descructor is called)
  return std::invoke(func, args...);
}

}  // namespace utils
}  // namespace aid4crop
