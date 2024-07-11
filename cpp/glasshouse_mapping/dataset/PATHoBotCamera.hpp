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

#include <open3d/camera/PinholeCameraIntrinsic.h>
#include <open3d/geometry/RGBDImage.h>

#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <opencv4/opencv2/core.hpp>
#include <vector>

namespace fs = std::filesystem;

namespace aid4crop {
namespace dataset {

class PATHoBotCamera {
 public:
  PATHoBotCamera(const fs::path&, const int = -1);

  size_t size() const;
  const open3d::camera::PinholeCameraIntrinsic& get_intrinsics() const;
  const Eigen::Matrix4d& get_extrinsics() const;
  std::shared_ptr<open3d::geometry::RGBDImage> get_image(const int&) const;
  std::tuple<cv::Mat, cv::Mat> get_opencv_image(const int&) const;
  const std::string get_name(const int& idx) const;

 private:
  void _load_camera_parameters(const fs::path&, const int& = 640,
                               const int& = 360);

  const fs::path _folder;
  std::vector<fs::path> _rgb_filenames;
  std::vector<fs::path> _depth_filenames;
  open3d::camera::PinholeCameraIntrinsic _intrinsics;
  Eigen::Matrix4d _extrinsics;
  size_t _size;
};

}  // namespace dataset
}  // namespace aid4crop
