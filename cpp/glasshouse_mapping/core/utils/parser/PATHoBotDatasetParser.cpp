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
#include "PATHoBotDatasetParser.hpp"

#include <filesystem>
#include <iostream>

namespace aid4crop {
namespace utils {

fs::path get_folder_name_from_number(const fs::path& dataset_folder,
                                     const int dataset_number) {
  fs::path folder_name = "";
  for (const fs::path& el : fs::directory_iterator(dataset_folder)) {
    if (fs::is_directory(el) &&
        el.filename().string()[0] == std::to_string(dataset_number)[0]) {
      folder_name = el.filename();
    }
  }

  // If we didn't find one exit with error
  if (folder_name.string() == "") {
    std::cout << "ERROR: No dataset found with number " << dataset_number
              << std::endl;
    exit(1);
  }

  return folder_name;
}

}  // namespace utils
}  // namespace aid4crop
