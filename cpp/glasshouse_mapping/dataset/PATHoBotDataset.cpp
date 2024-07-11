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
#include "PATHoBotDataset.hpp"

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace aid4crop::core;

namespace aid4crop {
namespace dataset {

PATHoBotDataset::PATHoBotDataset(const std::string& folder,
                                 const std::vector<Eigen::Matrix4d>& poses,
                                 const int n_frames_to_use,
                                 const int n_sensors_to_use)
    : _folder{folder} {
  // Initialization
  this->_initialize_dataset(folder, n_frames_to_use, n_sensors_to_use);

  // Save the given poses
  this->_poses = poses;

  // Check that the number of poses is consistent with the number of images in
  // the dataset
  assert(this->_poses.size() == this->_dataset_size);
}

PATHoBotDataset::PATHoBotDataset(const std::string& folder,
                                 const bool ref_dataset,
                                 const int n_frames_to_use,
                                 const int n_sensors_to_use)
    : _folder{folder} {
  // Initialization
  this->_initialize_dataset(folder, n_frames_to_use, n_sensors_to_use);

  // If we are dealing with a reference dataset, load poses
  this->_poses.reserve(this->_dataset_size);
  if (ref_dataset) {
    auto poses_file = this->_folder / "poses_kitty.txt";
    assert(fs::exists(poses_file));
    PATHoBotDataset::load_kitty_poses(poses_file, this->_poses,
                                      this->_dataset_size);
  } else {
    for (unsigned int i = 0; i < this->_dataset_size; ++i) {
      this->_poses.emplace_back(Eigen::Matrix4d::Identity());
    }
  }

  // Check that the number of poses is consistent with the number of images in
  // the dataset
  assert(this->_poses.size() == this->_dataset_size);
}

size_t PATHoBotDataset::size() const { return this->_dataset_size; }

size_t PATHoBotDataset::sensors_size() const { return this->_n_sensors; }

Eigen::Matrix4d PATHoBotDataset::get_pose(const int& idx) const {
  return this->_poses[idx];
}

const std::vector<Eigen::Matrix4d>& PATHoBotDataset::get_poses() const {
  return this->_poses;
}

void PATHoBotDataset::set_pose(const int& idx, Eigen::Matrix4d& pose) {
  this->_poses[idx] = pose;
}

std::tuple<std::vector<SensorInfo>, Eigen::Matrix4d>
PATHoBotDataset::get_element(const int& idx) const {
  std::vector<SensorInfo> frames;
  frames.reserve(this->_cameras.size());
  for (const auto& cam : this->_cameras) {
    frames.emplace_back(SensorInfo(cam.get_image(idx), cam.get_intrinsics(),
                                   cam.get_extrinsics()));
  }
  return {frames, this->get_pose(idx)};
}

std::tuple<std::vector<SensorInfo>, Eigen::Matrix4d>
PATHoBotDataset::get_element(const int& idx, const int& cam_idx) const {
  assert(cam_idx >= 0 && cam_idx < this->_cameras.size());
  return {{SensorInfo(this->_cameras[cam_idx].get_image(idx),
                      this->_cameras[cam_idx].get_intrinsics(),
                      this->_cameras[cam_idx].get_extrinsics())},
          this->get_pose(idx)};
}

const std::string PATHoBotDataset::get_element_name(const int& idx) const {
  return this->_cameras[0].get_name(idx);
}

std::vector<std::tuple<cv::Mat, cv::Mat>> PATHoBotDataset::get_opencv_element(
    const int& idx, const int& cam_idx) const {
  assert(cam_idx >= 0 && cam_idx < this->_cameras.size());
  return {this->_cameras[cam_idx].get_opencv_image(idx)};
}

const std::vector<std::tuple<const open3d::camera::PinholeCameraIntrinsic,
                             const Eigen::Matrix4d>>
PATHoBotDataset::get_cameras_info() const {
  // Initialize the ouput
  std::vector<std::tuple<const open3d::camera::PinholeCameraIntrinsic,
                         const Eigen::Matrix4d>>
      cameras_info;

  // Add the information for each sensor
  cameras_info.reserve(this->_cameras.size());
  for (const auto& cam : this->_cameras) {
    cameras_info.emplace_back(
        std::tuple<const open3d::camera::PinholeCameraIntrinsic,
                   const Eigen::Matrix4d>(cam.get_intrinsics(),
                                          cam.get_extrinsics()));
  }

  return cameras_info;
}  // namespace aid4crop

void PATHoBotDataset::load_kitty_poses(const std::string& filename,
                                       std::vector<Eigen::Matrix4d>& poses,
                                       const int max_n_poses) {
  std::ifstream infile(filename);
  std::string line;
  int n_loaded_poses = 0;
  while (n_loaded_poses < max_n_poses && std::getline(infile, line)) {
    Eigen::Matrix4d current_pose = Eigen::Matrix4d::Identity();
    std::istringstream iss(line);
    iss >> current_pose(0, 0) >> current_pose(0, 1) >> current_pose(0, 2) >>
        current_pose(0, 3) >> current_pose(1, 0) >> current_pose(1, 1) >>
        current_pose(1, 2) >> current_pose(1, 3) >> current_pose(2, 0) >>
        current_pose(2, 1) >> current_pose(2, 2) >> current_pose(2, 3);
    poses.emplace_back(current_pose);
    n_loaded_poses++;
  }
  infile.close();
}

void PATHoBotDataset::save_kitty_poses(
    const std::string& filename, const std::vector<Eigen::Matrix4d>& poses) {
  std::ofstream outfile(filename);
  for (const auto& el : poses) {
    std::ostringstream oss;
    oss << el(0, 0) << " " << el(0, 1) << " " << el(0, 2) << " " << el(0, 3)
        << " " << el(1, 0) << " " << el(1, 1) << " " << el(1, 2) << " "
        << el(1, 3) << " " << el(2, 0) << " " << el(2, 1) << " " << el(2, 2)
        << " " << el(2, 3) << std::endl;
    outfile << oss.str();
  }
  outfile.close();
}

void PATHoBotDataset::_initialize_dataset(const std::string& folder,
                                          const int n_frames_to_use,
                                          const int n_sensors_to_use) {
  // Check that the cameras folder exists
  fs::path cameras_folder = this->_folder / "cameras";
  assert(fs::is_directory(cameras_folder));

  // Initialize the cameras veryfing that we have at least of of them
  std::vector<fs::path> cams_folders;
  for (const auto& el : fs::directory_iterator(cameras_folder)) {
    cams_folders.emplace_back(el);
  }
  std::sort(cams_folders.begin(), cams_folders.end());
  if (n_sensors_to_use < 1 || n_sensors_to_use >= cams_folders.size()) {
    this->_n_sensors = cams_folders.size();
  } else {
    this->_n_sensors = n_sensors_to_use;
    cams_folders = std::vector<fs::path>(
        cams_folders.begin(), cams_folders.begin() + this->_n_sensors);
  }
  int sensor_idx = 0;
  this->_cameras.reserve(this->_n_sensors);
  for (const auto& el : cams_folders) {
    this->_cameras.emplace_back(PATHoBotCamera(el, n_frames_to_use));
    sensor_idx++;
  }
  assert(this->_cameras.size() > 0 &&
         this->_cameras.size() == this->_n_sensors);

  // Check that all the cameras contain the same number of elements
  if (n_frames_to_use < 1 || n_frames_to_use >= this->_cameras[0].size()) {
    this->_dataset_size = this->_cameras[0].size();
  } else {
    this->_dataset_size = n_frames_to_use;
  }
  for (const auto& el : this->_cameras) {
    assert(el.size() == this->_dataset_size);
  }
}

}  // namespace dataset
}  // namespace aid4crop
