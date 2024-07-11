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

#include <string>

namespace aid4crop {
namespace utils {

struct Config {
  const float map_voxel_size;
  const float registration_voxel_size;
  const float local_map_size;
  const int image_stride;
  const int first_frame;
  const int n_frames;
  const int n_sensors;
  const float depth_min_th;
  const float depth_max_th;
  const float icp_th;
  const int frame_frequency;
  const std::string superpoint_weights;
  const float ref_stable_features_min_th;
  const float ref_stable_features_max_th;
  const int first_frame_nicp;
  const int visual_match_frequency;
  const int deformation_graph_connectivity;
  const float deformation_graph_resolution;
  const float matches_cloud_resolution;
  const int deformation_frame_frequency;
  const float fpfh_voxel_size;
};

Config load_config(const std::string&);

}  // namespace utils
}  // namespace aid4crop
