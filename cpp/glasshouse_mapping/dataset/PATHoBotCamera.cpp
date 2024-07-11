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
#include "PATHoBotCamera.hpp"

#include <Eigen/src/Geometry/Quaternion.h>
#include <open3d/geometry/Image.h>
#include <open3d/geometry/RGBDImage.h>
#include <open3d/io/ImageIO.h>

#include <cassert>
#include <filesystem>
#include <iostream>
#include <memory>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace fs = std::filesystem;

namespace aid4crop {
namespace dataset {

PATHoBotCamera::PATHoBotCamera(const fs::path& folder,
                               const int n_frames_to_use)
    : _folder{folder} {
  // Load the camera parameters
  this->_load_camera_parameters(folder / "params.yaml");

  // Check that the rgb and depth subfolders exist
  fs::path rgb_folder(folder / "rgb");
  fs::path depth_folder(folder / "depth");
  assert(fs::is_directory(rgb_folder) && fs::is_directory(depth_folder));

  // Load the filenames from each folder
  for (const auto& el : fs::directory_iterator(rgb_folder)) {
    this->_rgb_filenames.emplace_back(el);
  }
  std::sort(this->_rgb_filenames.begin(), this->_rgb_filenames.end());
  this->_depth_filenames.reserve(this->_rgb_filenames.size());
  for (const auto& el : fs::directory_iterator(depth_folder)) {
    this->_depth_filenames.emplace_back(el);
  }
  std::sort(this->_depth_filenames.begin(), this->_depth_filenames.end());

  // Correct the number of frames (if needed) and check consistency
  if (n_frames_to_use < 1) {
    this->_size = this->_rgb_filenames.size();
  } else {
    this->_size = n_frames_to_use;
    assert(this->_rgb_filenames.size() > this->_size &&
           this->_depth_filenames.size() > this->_size);
    this->_rgb_filenames =
        std::vector<fs::path>(this->_rgb_filenames.begin(),
                              this->_rgb_filenames.begin() + this->_size);
    this->_depth_filenames =
        std::vector<fs::path>(this->_depth_filenames.begin(),
                              this->_depth_filenames.begin() + this->_size);
  }
  assert(this->_rgb_filenames.size() == this->_depth_filenames.size() &&
         this->_size == this->_rgb_filenames.size());
}

void PATHoBotCamera::_load_camera_parameters(const fs::path& filename,
                                             const int& img_width,
                                             const int& img_height) {
  // Load the file
  YAML::Node cam_params = YAML::LoadFile(filename.generic_string());

  // Initialize the intrinsics
  this->_intrinsics = open3d::camera::PinholeCameraIntrinsic(
      img_width, img_height, cam_params["intrinsics"][0][0].as<float>(),
      cam_params["intrinsics"][1][1].as<float>(),
      cam_params["intrinsics"][0][2].as<float>(),
      cam_params["intrinsics"][1][2].as<float>());

  // Initialize the extrinsics
  this->_extrinsics = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q{cam_params["extrinsics"][1][3].as<float>(),
                       cam_params["extrinsics"][1][0].as<float>(),
                       cam_params["extrinsics"][1][1].as<float>(),
                       cam_params["extrinsics"][1][2].as<float>()};
  this->_extrinsics.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  this->_extrinsics.block<3, 1>(0, 3) =
      Eigen::Vector3d(cam_params["extrinsics"][0][0].as<float>(),
                      cam_params["extrinsics"][0][1].as<float>(),
                      cam_params["extrinsics"][0][2].as<float>());
}

const Eigen::Matrix4d& PATHoBotCamera::get_extrinsics() const {
  return this->_extrinsics;
}

const open3d::camera::PinholeCameraIntrinsic& PATHoBotCamera::get_intrinsics()
    const {
  return this->_intrinsics;
}

std::shared_ptr<open3d::geometry::RGBDImage> PATHoBotCamera::get_image(
    const int& idx) const {
  open3d::geometry::Image rgb_img, depth_img;
  open3d::io::ReadImage(this->_rgb_filenames[idx], rgb_img);
  open3d::io::ReadImage(this->_depth_filenames[idx], depth_img);
  return open3d::geometry::RGBDImage::CreateFromColorAndDepth(
      rgb_img, depth_img, 1000.0, 3.0, false);
}

std::tuple<cv::Mat, cv::Mat> PATHoBotCamera::get_opencv_image(
    const int& idx) const {
  return {cv::imread(this->_rgb_filenames[idx], cv::IMREAD_COLOR),
          cv::imread(this->_depth_filenames[idx], cv::IMREAD_ANYDEPTH)};
}

const std::string PATHoBotCamera::get_name(const int& idx) const {
  return this->_rgb_filenames[idx].stem();
}

size_t PATHoBotCamera::size() const { return this->_size; }

}  // namespace dataset
}  // namespace aid4crop
