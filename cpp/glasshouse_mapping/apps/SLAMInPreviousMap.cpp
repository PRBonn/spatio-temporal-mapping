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
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <string>

#include "VoxelizedPointCloud.hpp"
#include "core/Localization.hpp"
#include "core/Mapping.hpp"
#include "dataset/PATHoBotDataset.hpp"
#include "utils/config/Config.hpp"
#include "utils/parser/CLI11.hpp"
#include "utils/parser/PATHoBotDatasetParser.hpp"

using namespace aid4crop::dataset;
using namespace aid4crop::core;
using namespace aid4crop::utils;
namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
  // Parse the command line
  CLI::App app{
      "Mapping and localization system esploiting previous map. This system "
      "produce a map aligned with a reference map (assuming that they both "
      "model the same environment)."};
  std::string dataset_main_folder = "dataset/";
  std::string cfg_filename = "config/config.yaml";
  int ref_number = 1;
  int query_number = 2;
  int row_number = 3;
  bool use_wheel_odometry = true;
  bool visualize = false;
  app.add_option(
      "-d,--dataset-folder", dataset_main_folder,
      "The path to the folder where it is contained the dataset to deal with.");
  app.add_option("-r,--ref-number", ref_number,
                 "Number of the dataset to use as reference");
  app.add_option("-q,--query-number", query_number,
                 "Number of the dataset to use as query");
  app.add_option("--row-number", row_number, "Number of the row to use");
  app.add_option("--use-wheel-odometry", use_wheel_odometry,
                 "If you want to use or \
      not wheel odometry as initial guess for each pose during visual odometry.");
  app.add_option("--config-filename", cfg_filename,
                 "Path to the configuration file.");
  app.add_option("-v,--visualize", visualize, "Visualization ON/OFF");
  CLI11_PARSE(app, argc, argv);

  // Load the configuration file
  Config cfg = load_config(cfg_filename);

  // Initialize the name of the folders to use
  fs::path ref_number_folder =
      get_folder_name_from_number(dataset_main_folder, ref_number);
  fs::path query_number_folder =
      get_folder_name_from_number(dataset_main_folder, query_number);
  fs::path ref_dataset_folder = dataset_main_folder / ref_number_folder /
                                ("row" + std::to_string(row_number));
  fs::path query_dataset_folder = dataset_main_folder / query_number_folder /
                                  ("row" + std::to_string(row_number));
  fs::path ref_map_filename =
      ref_dataset_folder / "mapping_out/voxelized_map.ply";
  fs::path ref_poses_filename =
      ref_dataset_folder / "mapping_out/mapping_poses.txt";
  fs::path output_folder = query_dataset_folder / "mapping_out";
  if (!fs::exists(output_folder)) {
    fs::create_directory(output_folder);
  }
  /* fs::path output_map_filename = */
  /*     "slam_voxelized_map_ref_" + std::to_string(ref_number) + ".ply"; */
  /* fs::path output_poses_filename = */
  /*     "localization_poses_ref_" + std::to_string(ref_number) + ".txt"; */
  fs::path output_map_filename =
      "voxelized_map_onref_" + std::to_string(ref_number) + ".ply";
  fs::path output_poses_filename =
      "mapping_poses_onref_" + std::to_string(ref_number) + ".txt";

  // Initialize current dataset and some variable
  PATHoBotDataset current_dataset(query_dataset_folder, true, -1,
                                  cfg.n_sensors);
  bool global_relocalization_needed = true;

  // Load the reference dataset
  auto ref_map_complete =
      VoxelizedPointCloud::load(ref_map_filename, cfg.registration_voxel_size);
  auto local_map = VoxelizedPointCloud(cfg.registration_voxel_size);
  auto output_map = VoxelizedPointCloud(cfg.map_voxel_size);
  std::vector<Eigen::Matrix4d> ref_poses;
  PATHoBotDataset::load_kitty_poses(ref_poses_filename, ref_poses);
  assert(cfg.n_frames < 0 || ref_poses.size() >= cfg.n_frames);
  if (cfg.n_frames >= 0)
    ref_poses = {ref_poses.begin(), ref_poses.begin() + cfg.n_frames};
  PATHoBotDataset ref_dataset(ref_dataset_folder, ref_poses, cfg.n_frames,
                              cfg.n_sensors);

  // Pre-process the reference map to speed up localization
  auto ref_map = ref_map_complete.Threshold(2, cfg.ref_stable_features_min_th,
                                            cfg.ref_stable_features_max_th);
  ref_map.EstimateNormals();

  // Initialize poses book-keeping
  Eigen::Matrix4d previous_wheel_odom_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d previous_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d previous_previous_pose = Eigen::Matrix4d::Identity();
  float current_velocity = 0.0;

  // Initialize stuff to measure frame execution time
  std::chrono::time_point<std::chrono::high_resolution_clock> t1, t2;

  // For each frame in the current dataset compute the pose in the reference map
  const size_t current_dataset_len = current_dataset.size();
  for (int current_frame_idx = 0; current_frame_idx < current_dataset_len;
       ++current_frame_idx) {
    // LOGGING
    std::cout << std::endl << "Frame " << current_frame_idx << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    // Get the current element from the dataset
    auto [sensors, wheel_odom_pose] =
        current_dataset.get_element(current_frame_idx, 0);

    // Get an initial guess on the pose
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    if (global_relocalization_needed) {
      bool succ = false;
      initial_guess = ref_dataset.get_poses().front();
      // Initialize local_map
      local_map =
          ref_map.Resize((initial_guess * sensors[0].camera_extrinsics)(0, 3),
                         cfg.local_map_size);
      previous_pose = initial_guess;
      global_relocalization_needed = false;
    } else if (use_wheel_odometry) {
      std::tie(initial_guess, current_velocity) =
          integrate_wheel_odometry(wheel_odom_pose, previous_wheel_odom_pose,
                                   previous_pose, current_velocity);
    } else {
      std::tie(initial_guess, current_velocity) =
          constant_velocity_model(previous_pose, previous_previous_pose);
    }

    // Extract the point cloud
    auto new_pcd = extract_point_cloud(
        sensors, initial_guess, cfg.map_voxel_size, cfg.depth_min_th,
        cfg.depth_max_th, cfg.image_stride, 0, -1);
    auto new_pcd_registration =
        VoxelizedPointCloud(cfg.registration_voxel_size);
    new_pcd_registration.IntegratePointCloud(new_pcd.get_pcd());
    new_pcd_registration = new_pcd_registration.Threshold(
        2, cfg.ref_stable_features_min_th, cfg.ref_stable_features_max_th);

    // Use ICP to estimate the real pose w.r.t. local ref_map
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    new_pcd.Transform(initial_guess.inverse());
    new_pcd_registration.Transform(initial_guess.inverse());
    std::tie(pose, std::ignore) = correct_pose_with_icp(
        new_pcd_registration, local_map.get_pcd(), initial_guess, cfg.icp_th);
    new_pcd.Transform(pose);
    new_pcd_registration.Transform(pose);

    // With the given frequency update the maps
    if (current_frame_idx == 0 ||
        (current_frame_idx % cfg.frame_frequency) == 0) {
      output_map.IntegratePointCloud(new_pcd.get_pcd());
      local_map.IntegratePointCloud(new_pcd_registration.get_pcd());
      local_map = local_map.Resize((pose * sensors[0].camera_extrinsics)(0, 3),
                                   cfg.local_map_size);
      local_map.IntegratePointCloud(
          ref_map
              .Resize((pose * sensors[0].camera_extrinsics)(0, 3),
                      cfg.local_map_size)
              .get_pcd());
    }

    // Update the dataset pose
    current_dataset.set_pose(current_frame_idx, pose);

    // Stop when we reach the number of frames to deal with
    if (current_frame_idx == cfg.n_frames - 1) {
      break;
    }

    // Save information for the next frame
    previous_wheel_odom_pose = wheel_odom_pose;
    previous_previous_pose = previous_pose;
    previous_pose = pose;

    // LOGGING
    std::cout << "Frame execution time: " << std::fixed
              << std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::high_resolution_clock::now() - t1)
                         .count() *
                     1e-9
              << "sec" << std::endl
              << std::setprecision(5);
  }

  // Save the computed poses and map
  VoxelizedPointCloud::save(output_folder / output_map_filename, output_map);
  PATHoBotDataset::save_kitty_poses(output_folder / output_poses_filename,
                                    current_dataset.get_poses());

  // Visualize the computed map
  if (visualize) {
    ref_map_complete.visualize(output_map.get_pcd());
    output_map.visualize();
  }

  return 0;
}
