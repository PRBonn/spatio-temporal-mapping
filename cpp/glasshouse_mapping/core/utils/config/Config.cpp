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
#include "Config.hpp"

#include <iostream>

#include "yaml-cpp/yaml.h"

namespace aid4crop {
namespace utils {

Config load_config(const std::string& cfg_filename) {
  YAML::Node cfg_file = YAML::LoadFile(cfg_filename);
  return Config{
      cfg_file["mapping"]["map_voxel_size"].as<float>(),
      cfg_file["mapping"]["registration_voxel_size"].as<float>(),
      cfg_file["mapping"]["local_map_size"].as<float>(),
      cfg_file["mapping"]["image_stride"].as<int>(),
      cfg_file["general"]["first_frame"].as<int>(),
      cfg_file["general"]["n_frames"].as<int>(),
      cfg_file["general"]["n_sensors"].as<int>(),
      cfg_file["general"]["depth_min_th"].as<float>(),
      cfg_file["general"]["depth_max_th"].as<float>(),
      cfg_file["general"]["icp_th"].as<float>(),
      cfg_file["general"]["frame_frequency"].as<int>(),
      cfg_file["general"]["superpoint_weights"].as<std::string>(),
      cfg_file["slam"]["ref_stable_features_min_th"].as<float>(),
      cfg_file["slam"]["ref_stable_features_max_th"].as<float>(),
      cfg_file["non_rigid_icp"]["first_frame_nicp"].as<int>(),
      cfg_file["non_rigid_icp"]["visual_match_frequency"].as<int>(),
      cfg_file["non_rigid_icp"]["deformation_graph_connectivity"].as<int>(),
      cfg_file["non_rigid_icp"]["deformation_graph_resolution"].as<float>(),
      cfg_file["non_rigid_icp"]["matches_cloud_resolution"].as<float>(),
      cfg_file["non_rigid_icp"]["deformation_frame_frequency"].as<int>(),
      cfg_file["baselines"]["fpfh_voxel_size"].as<float>()};
}

}  // namespace utils
}  // namespace aid4crop
