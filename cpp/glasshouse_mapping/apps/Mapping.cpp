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
#include "core/Mapping.hpp"

#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <string>
#include <tuple>

#include "VoxelizedPointCloud.hpp"
#include "core/Localization.hpp"
#include "dataset/PATHoBotDataset.hpp"
#include "utils/config/Config.hpp"
#include "utils/decorators/ExecutionTimerDecorator.hpp"
#include "utils/parser/CLI11.hpp"
#include "utils/parser/PATHoBotDatasetParser.hpp"
#include "yaml-cpp/yaml.h"

using namespace aid4crop::dataset;
using namespace aid4crop::core;
using namespace aid4crop::utils;
namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
  // Parse the command line
  CLI::App app{
      "Mapping system. This system produce a map given a set of images."};
  std::string dataset_main_folder = "dataset/";
  std::string cfg_filename = "config/config.yaml";
  int dataset_number = 1;
  int row_number = 2;
  bool visual_odometry = true;
  bool use_wheel_odometry = true;
  bool visualize = false;
  app.add_option(
      "-d,--dataset-folder", dataset_main_folder,
      "The path to the folder where it is contained the dataset to deal with.");
  app.add_option("-n,--dataset-number", dataset_number,
                 "Number of the dataset to use for mapping");
  app.add_option("-r,--row-number", row_number, "Number of the row to use");
  app.add_option("--visual-odometry", visual_odometry,
                 "If we want to perform \
      visual odometry (so we do not have correct pose for this dataset) or we \
      want to use previous computed poses for mapping.");
  app.add_option("--use-wheel-odometry", use_wheel_odometry,
                 "If you want to use or \
      not wheel odometry as initial guess for each pose during visual odometry.");
  app.add_option("--config-filename", cfg_filename,
                 "Path to the configuration file.");
  app.add_option("-v,--visualize", visualize, "Visualization ON/OFF");
  CLI11_PARSE(app, argc, argv);

  // Load the configuration file
  Config cfg = load_config(cfg_filename);

  // Initialize the name of the folder to use
  fs::path dataset_number_folder =
      get_folder_name_from_number(dataset_main_folder, dataset_number);
  fs::path dataset_folder = dataset_main_folder / dataset_number_folder /
                            ("row" + std::to_string(row_number));
  fs::path output_folder = dataset_folder / "mapping_out";
  fs::path poses_filename = output_folder / "localization_poses.txt";
  if (!fs::exists(output_folder)) {
    fs::create_directory(output_folder);
  }

  // Initialize dataset and maps
  PATHoBotDataset dataset(dataset_folder, true, cfg.n_frames, cfg.n_sensors);
  auto output_map = VoxelizedPointCloud(cfg.map_voxel_size);
  auto local_map = VoxelizedPointCloud(cfg.registration_voxel_size);

  // If we are not in visual odometry mode, load the poses
  std::vector<Eigen::Matrix4d> poses;
  if (!visual_odometry) {
    PATHoBotDataset::load_kitty_poses(poses_filename, poses);
  }

  // Initialize poses book-keeping
  Eigen::Matrix4d previous_pose_raw = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d previous_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d previous_previous_pose = Eigen::Matrix4d::Identity();
  float current_velocity = 0.0;

  // Initialize stuff to measure frame execution time
  std::chrono::time_point<std::chrono::high_resolution_clock> t1, t2;

  // Extract map from each frame in the dataset_clean
  const int dataset_len = dataset.size();
  for (int frame_idx = 0; frame_idx < dataset_len; ++frame_idx) {
    // LOGGING
    std::cout << std::endl << "Frame " << frame_idx << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    // Get the current element from the dataset
    auto [sensors, pose_raw] = dataset.get_element(frame_idx, 0);

    // Compute the pose if we are in visual odometry mode
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    if (visual_odometry && frame_idx > 0) {
      if (use_wheel_odometry) {
        std::tie(pose, current_velocity) = integrate_wheel_odometry(
            pose_raw, previous_pose_raw, previous_pose, current_velocity);
      } else {
        std::tie(pose, current_velocity) =
            constant_velocity_model(previous_pose, previous_previous_pose);
      }
    } else if (visual_odometry) {
      if (use_wheel_odometry) {
        pose = pose_raw;
      } else {
        pose = Eigen::Matrix4d::Identity();
      }
    } else {
      pose = poses[frame_idx];
    }

    // Extract the point cloud
    auto new_pcd =
        extract_point_cloud(sensors, pose, cfg.map_voxel_size, cfg.depth_min_th,
                            cfg.depth_max_th, cfg.image_stride, 0, -1);
    auto new_pcd_registration =
        VoxelizedPointCloud(cfg.registration_voxel_size);
    new_pcd_registration.IntegratePointCloud(new_pcd.get_pcd());

    // If we are in visual odometry mode, correct the pose with ICP
    if (visual_odometry && frame_idx > 0) {
      new_pcd.Transform(pose.inverse());
      new_pcd_registration.Transform(pose.inverse());
      std::tie(pose, std::ignore) = correct_pose_with_icp(
          new_pcd_registration, local_map.get_pcd(), pose, cfg.icp_th);
      new_pcd.Transform(pose);
      new_pcd_registration.Transform(pose);
    }

    // With a given frequency
    if (frame_idx == 0 || (frame_idx % cfg.frame_frequency) == 0) {
      // Add the new points to the map every N frames
      output_map.IntegratePointCloud(new_pcd.get_pcd());
      local_map.IntegratePointCloud(new_pcd_registration.get_pcd());

      // Reduce the local map
      local_map = local_map.Resize((pose * sensors[0].camera_extrinsics)(0, 3),
                                   cfg.local_map_size);
    }

    // Save information for the next frame (in visual odometry mode)
    if (visual_odometry) {
      previous_pose_raw = pose_raw;
      previous_previous_pose = previous_pose;
      previous_pose = pose;
    }

    // Update the pose (in visual odometry pose)
    if (visual_odometry) {
      dataset.set_pose(frame_idx, pose);
    }

    // LOGGING
    std::cout << "Frame execution time: " << std::fixed
              << std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::high_resolution_clock::now() - t1)
                         .count() *
                     1e-9
              << "sec" << std::endl
              << std::setprecision(5);
  }

  // Save the map and the poses
  VoxelizedPointCloud::save(output_folder / "voxelized_map.ply", output_map);
  PATHoBotDataset::save_kitty_poses(output_folder / "mapping_poses.txt",
                                    dataset.get_poses());

  // Visualize the output map
  if (visualize) output_map.visualize();
}
