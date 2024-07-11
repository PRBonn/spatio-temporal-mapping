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
#include <open3d/utility/Eigen.h>

#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include "PATHoBotCamera.hpp"
#include "SensorInfo.hpp"

namespace fs = std::filesystem;
using SensorInfo = aid4crop::core::SensorInfo;

namespace aid4crop {

namespace dataset {

class PATHoBotDataset {
 public:
  PATHoBotDataset(const std::string&, const bool = true, const int = -1,
                  const int = -1);
  PATHoBotDataset(const std::string&, const std::vector<Eigen::Matrix4d>& poses,
                  const int = -1, const int = -1);

  size_t size() const;
  size_t sensors_size() const;
  Eigen::Matrix4d get_pose(const int&) const;
  const std::vector<Eigen::Matrix4d>& get_poses() const;
  void set_pose(const int&, Eigen::Matrix4d&);
  std::tuple<std::vector<SensorInfo>, Eigen::Matrix4d> get_element(
      const int&) const;
  std::tuple<std::vector<SensorInfo>, Eigen::Matrix4d> get_element(
      const int&, const int&) const;
  const std::string get_element_name(const int&) const;

  std::vector<std::tuple<cv::Mat, cv::Mat>> get_opencv_element(
      const int&, const int&) const;

  const std::vector<std::tuple<const open3d::camera::PinholeCameraIntrinsic,
                               const Eigen::Matrix4d>>
  get_cameras_info() const;

  static void load_kitty_poses(const std::string&,
                               std::vector<Eigen::Matrix4d>&,
                               const int = std::numeric_limits<int>::max());
  static void save_kitty_poses(const std::string&,
                               const std::vector<Eigen::Matrix4d>&);

 private:
  void _initialize_dataset(const std::string&, const int, const int);

  const fs::path _folder;
  std::vector<PATHoBotCamera> _cameras;
  size_t _dataset_size;
  size_t _n_sensors;
  std::vector<Eigen::Matrix4d> _poses;
};

}  // namespace dataset
}  // namespace aid4crop
