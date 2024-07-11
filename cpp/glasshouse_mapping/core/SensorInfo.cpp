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
#include <SensorInfo.hpp>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ranges>
#include <sstream>

namespace fs = std::filesystem;

namespace aid4crop {
namespace core {

SensorInfo::SensorInfo(const std::shared_ptr<open3d::geometry::RGBDImage>& img,
                       const open3d::camera::PinholeCameraIntrinsic& intrinsics,
                       const Eigen::Matrix4d& extrinsics)
    : rgbd_img{img},
      camera_intrinsics{intrinsics},
      camera_extrinsics{extrinsics} {}

std::vector<std::vector<Measurements>> load_measurements(
    const std::string& filename, const size_t n_frames,
    const size_t n_sensors) {
  // Initialization
  std::vector<std::vector<Measurements>> measurements(
      n_frames, std::vector<Measurements>(n_sensors, Measurements()));

  // Load info from the file (IMPORTANT: we need to know exactly #frames and
  // #sensors in order to load without errors)
  std::ifstream infile(filename);
  std::string line;
  size_t frame_idx;
  size_t sensor_idx;
  while (std::getline(infile, line)) {
    Measurements sensor_measurements;
    std::istringstream iss(line);
    double u, v, z;
    while (iss >> u >> v >> z) {
      sensor_measurements.emplace_back(Eigen::Vector3d(u, v, z));
    }
    measurements[frame_idx][sensor_idx++] = sensor_measurements;
    if (sensor_idx >= n_sensors) {
      sensor_idx = 0;
      frame_idx++;
    }
    if (frame_idx > n_frames) {
      break;
    }
  }
  infile.close();

  return measurements;
}

void save_measurements(
    const std::string& filename,
    const std::vector<std::vector<Measurements>>& measurements) {
  // Initialization
  const size_t n_sensors = measurements[0].size();

  // Save info in a file (IMPORTANT: to load this you should know exactly the
  // number of sensors and frames used)
  std::ofstream outfile(filename);
  for (const auto& frame : measurements) {
    for (const auto& sensor : frame) {
      std::ostringstream oss;
      for (const auto& meas : sensor) {
        oss << meas(0) << " " << meas(1) << " " << meas(2) << " ";
      }
      oss << std::endl;
      outfile << oss.str();
    }
  }
  outfile.close();
}

}  // namespace core
}  // namespace aid4crop
