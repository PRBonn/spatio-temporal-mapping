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
#include "Localization.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/RobustKernel.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

#include "VoxelizedPointCloud.hpp"

namespace fs = std::filesystem;

namespace aid4crop {
namespace core {

std::tuple<Eigen::Matrix4d, float> integrate_wheel_odometry(
    const Eigen::Matrix4d& wheel_odom_pose_raw,
    const Eigen::Matrix4d& previous_wheel_odom_pose_raw,
    const Eigen::Matrix4d& previous_pose, const float velocity,
    const float delta_th) {
  // Initialize the wheel odometry to the previous pose
  auto wheel_odom_pose = previous_pose;

  // Compute the delta between the two consecutive wheel odometries on the x
  // axis
  float delta_x =
      wheel_odom_pose_raw(0, 3) - previous_wheel_odom_pose_raw(0, 3);

  // If the delta is over a theshold then there is an error in it and we need to
  // correct with the constant velocity model
  float new_velocity;
  if (abs(delta_x) > delta_th) {
    // Use constant velocity model
    wheel_odom_pose(0, 3) += velocity;

    // Keep the velocity as before
    new_velocity = velocity;
  } else {
    // Otherwise integrate the wheel odometry delta
    wheel_odom_pose(0, 3) += delta_x;

    // Update the current velocity
    new_velocity = delta_x;
  }

  return {wheel_odom_pose, new_velocity};
}

std::tuple<Eigen::Matrix4d, float> constant_velocity_model(
    const Eigen::Matrix4d& previous_pose,
    const Eigen::Matrix4d& previous_previous_pose) {
  // Initialize the new pose as the previous pose
  Eigen::Matrix4d pose = previous_pose;

  // Compute the velocity of the previous two poses on the x axis
  float velocity = previous_pose(0, 3) - previous_previous_pose(0, 3);

  // Increment the current pose by the given velocity
  pose(0, 3) += velocity;

  return {pose, velocity};
}

open3d::pipelines::registration::CorrespondenceSet
remove_duplicate_correspondences(
    open3d::pipelines::registration::CorrespondenceSet& correspondences) {
  // Initialization
  open3d::pipelines::registration::CorrespondenceSet filtered_correspondences;

  // Sort correspondences by the second element
  std::sort(correspondences.begin(), correspondences.end(),
            [](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) -> bool {
              return lhs(1) < rhs(1);
            });

  // Filter duplicates
  int last_id = -1;
  filtered_correspondences.reserve(correspondences.size());
  for (const auto& el : correspondences) {
    if (el(1) != last_id) {
      filtered_correspondences.emplace_back(el);
      last_id = el(1);
    }
  }
  filtered_correspondences.shrink_to_fit();

  return filtered_correspondences;
}

std::tuple<Eigen::Matrix4d, open3d::pipelines::registration::CorrespondenceSet>
correct_pose_with_icp(const VoxelizedPointCloud& current_pcd,
                      const open3d::geometry::PointCloud& map_pcd,
                      const Eigen::Matrix4d& initial_guess,
                      const float icp_th) {
  // ICP
  auto registration_result =
      open3d::pipelines::registration::RegistrationGeneralizedICP(
          current_pcd.get_pcd(), map_pcd, icp_th, initial_guess,
          open3d::pipelines::registration::
              TransformationEstimationForGeneralizedICP(
                  1e-3,
                  std::make_shared<open3d::pipelines::registration::GMLoss>(
                      icp_th / 6)),
          open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-5,
                                                                  2000));

  // Take the resulting pose
  auto T = registration_result.transformation_;

  return {T, remove_duplicate_correspondences(
                 registration_result.correspondence_set_)};
}

std::tuple<Eigen::Matrix4d, bool> global_relocalization(
    const int query_id, const std::vector<int>& query_id2ref_id,
    const std::vector<Eigen::Matrix4d>& ref_poses) {
  // Initialization
  const size_t n_query_ids = query_id2ref_id.size();
  const size_t n_ref_poses = ref_poses.size();

  // Get the result of VPR
  assert(query_id >= 0);
  assert(query_id < n_query_ids);
  const int ref_id = query_id2ref_id[query_id];

  // Check if we have a result for the given query
  if (ref_id == -1 || ref_id < 0 || ref_id >= n_ref_poses) {
    return {Eigen::Matrix4d::Identity(), false};
  }

  // If it is valid return the given pose
  return {ref_poses[ref_id], true};
}

int match_images_by_pose(const Eigen::Matrix4d& query_pose,
                         const std::vector<Eigen::Matrix4d>& ref_poses) {
  int best_idx = -1;
  int current_idx = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  double current_distance;
  for (const auto& ref_pose : ref_poses) {
    current_distance =
        std::pow((query_pose.row(0)[3] - ref_pose.row(0)[3]), 2) +
        std::pow((query_pose.row(1)[3] - ref_pose.row(1)[3]), 2) +
        std::pow((query_pose.row(2)[3] - ref_pose.row(2)[3]), 2);
    if (current_distance < best_distance) {
      best_distance = current_distance;
      best_idx = current_idx;
    }
    current_idx++;
  }
  assert(best_idx >= 0 and best_idx < ref_poses.size());
  return best_idx;
}

std::vector<int> load_vpr_results(const std::string& filename,
                                  const size_t query_dataset_size) {
  // Initialization
  std::vector<int> query_id2ref_id(query_dataset_size, -1);
  int query_idx, ref_idx;

  // Load the vpr results
  assert(fs::exists(filename));
  std::ifstream infile(filename);
  std::string line;
  while (std::getline(infile, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream iss(line);
    iss >> query_idx >> ref_idx;
    assert(query_idx >= 0 && query_idx < query_id2ref_id.size());
    query_id2ref_id[query_idx] = ref_idx;
  }
  infile.close();

  return query_id2ref_id;
}

}  // namespace core
}  // namespace aid4crop
