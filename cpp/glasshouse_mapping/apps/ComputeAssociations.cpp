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
#include <fstream>
#include <opencv4/opencv2/opencv.hpp>

#include "VoxelizedPointCloud.hpp"
#include "core/Mapping.hpp"
#include "utils/superpoint/superpoint_matcher.hpp"
#include "core/Localization.hpp"
#include "dataset/PATHoBotDataset.hpp"
#include "utils/config/Config.hpp"
#include "utils/nanoflann/nanoflannWrapper.hpp"
#include "utils/parser/CLI11.hpp"
#include "utils/parser/PATHoBotDatasetParser.hpp"
#include "utils/superpoint/superpoint.hpp"

using namespace aid4crop::dataset;
using namespace aid4crop::core;
using namespace aid4crop::utils;
namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
  // Parse the command line
  CLI::App app{
      "Odometry using a previous map and deformation of the same in order to "
      "obtain the new plants shhape."};
  std::string dataset_main_folder = "dataset/";
  std::string cfg_filename = "config/config.yaml";
  std::string viewpoint_filename = "config/viewpoint.json";
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
  fs::path gt_map_filename =
      query_dataset_folder / "mapping_out/voxelized_map.ply";
  fs::path output_folder = query_dataset_folder / "mapping_out";
  if (!fs::exists(output_folder)) {
    fs::create_directory(output_folder);
  }
  fs::path output_poses_filename =
      "localization_poses_onref_" + std::to_string(ref_number) + ".txt";
  fs::path output_matches_filename =
      "matches_onref_" + std::to_string(ref_number) + ".txt";

  // Load the reference dataset
  auto ref_map_complete =
      VoxelizedPointCloud::load(ref_map_filename, cfg.map_voxel_size);
  auto local_map = VoxelizedPointCloud(cfg.registration_voxel_size);
  std::vector<Eigen::Matrix4d> ref_poses;
  PATHoBotDataset::load_kitty_poses(ref_poses_filename, ref_poses);
  assert(cfg.n_frames < 0 || ref_poses.size() >= cfg.n_frames);
  if (cfg.n_frames >= 0)
    ref_poses = {ref_poses.begin(), ref_poses.begin() + cfg.n_frames};
  PATHoBotDataset ref_dataset(ref_dataset_folder, ref_poses, cfg.n_frames,
                              cfg.n_sensors);

  // Pre-process reference map to speed up localization
  VoxelizedPointCloud ref_map_stable_feats =
      VoxelizedPointCloud(cfg.registration_voxel_size);
  ref_map_stable_feats.IntegratePointCloud(
      ref_map_complete
          .Threshold(2, cfg.ref_stable_features_min_th,
                     cfg.ref_stable_features_max_th)
          .get_pcd());
  ref_map_stable_feats.EstimateNormals();

  // Initialize current dataset and some variable
  PATHoBotDataset current_dataset(query_dataset_folder, true, -1,
                                  cfg.n_sensors);

  // Initialize poses book-keeping
  Eigen::Matrix4d previous_wheel_odom_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d previous_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d previous_previous_pose = Eigen::Matrix4d::Identity();
  float current_velocity = 0.0;

  // Initialize stuff to measure frame execution time
  std::chrono::time_point<std::chrono::high_resolution_clock> t1, t2;

  // Initialize stuff for visual matching
  const SuperPoint superpoint = (cfg.superpoint_weights);
  std::vector<bool> is_point_associated(ref_map_complete.size(), false);
  std::vector<Eigen::Vector3d> source_points, target_points;
  auto ref_kdtree = nanoflann_wrapper(ref_map_complete.get_cloud());

  // For each frame in the current dataset compute the odometry and the
  // deformation
  const size_t current_dataset_len = current_dataset.size();
  for (int current_frame_idx = 0; current_frame_idx < current_dataset_len;
       ++current_frame_idx) {
    // LOGGING
    std::cout << std::endl << "Frame " << current_frame_idx << std::endl;
    t1 = std::chrono::high_resolution_clock::now();

    // ######### ODOMETRY #########

    // Get the current element from the dataset
    auto [sensors, wheel_odom_pose] =
        current_dataset.get_element(current_frame_idx, 0);

    // Get an initial guess on the pose
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    if (current_frame_idx == 0) {
      // ASSUMPTION: Here we assume that the starting position is the more or
      // less the same in the two sequences
      initial_guess = ref_dataset.get_poses().front();
    } else {
      std::tie(initial_guess, current_velocity) =
          integrate_wheel_odometry(wheel_odom_pose, previous_wheel_odom_pose,
                                   previous_pose, current_velocity);
    }
    previous_pose = initial_guess;

    // Update local map
    local_map = ref_map_stable_feats.Resize(
        (initial_guess * sensors[0].camera_extrinsics)(0, 3),
        cfg.local_map_size);

    // Extract the point cloud
    auto new_pcd =
        extract_point_cloud(sensors, initial_guess, cfg.registration_voxel_size,
                            cfg.depth_min_th, cfg.depth_max_th, 2, 0, -1);
    auto new_pcd_stable_feats =
        VoxelizedPointCloud(cfg.registration_voxel_size);
    new_pcd_stable_feats.IntegratePointCloud(new_pcd.get_pcd());
    new_pcd_stable_feats = new_pcd.Threshold(2, cfg.ref_stable_features_min_th,
                                             cfg.ref_stable_features_max_th);

    // Use ICP to estimate the real pose w.r.t. local ref_map
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    new_pcd.Transform(initial_guess.inverse());
    new_pcd_stable_feats.Transform(initial_guess.inverse());
    std::tie(pose, std::ignore) = correct_pose_with_icp(
        new_pcd_stable_feats, local_map.get_pcd(), initial_guess, cfg.icp_th);
    new_pcd.Transform(pose);
    new_pcd_stable_feats.Transform(pose);

    // Update the local map
    local_map.IntegratePointCloud(new_pcd_stable_feats.get_pcd());
    local_map = local_map.Resize((pose * sensors[0].camera_extrinsics)(0, 3),
                                 cfg.local_map_size);
    local_map.IntegratePointCloud(
        ref_map_stable_feats
            .Resize((pose * sensors[0].camera_extrinsics)(0, 3),
                    cfg.local_map_size)
            .get_pcd());

    // Update the dataset pose
    current_dataset.set_pose(current_frame_idx, pose);

    // Save information for the next frame
    previous_wheel_odom_pose = wheel_odom_pose;
    previous_previous_pose = previous_pose;
    previous_pose = pose;

    // ####### VISUAL MATCH #######
    // Compute image associations every N frames
    if (current_frame_idx == 0 ||
        (current_frame_idx % cfg.visual_match_frequency == 0)) {
      // Image association by pose
      const int ref_frame_idx =
          match_images_by_pose(pose, ref_dataset.get_poses());

      // Load the OpenCV version of the images
      auto [ref_rgb_img, ref_depth_img] =
          ref_dataset.get_opencv_element(ref_frame_idx, 0)[0];
      auto ref_sensor =
          std::get<0>(ref_dataset.get_element(ref_frame_idx, 0))[0];
      auto [query_rgb_img, query_depth_img] =
          current_dataset.get_opencv_element(current_frame_idx, 0)[0];

      // SuperPoint matches
      auto [kps1, dsc1] = superpoint.getFeatures(query_rgb_img);
      auto [kps2, dsc2] = superpoint.getFeatures(ref_rgb_img);
      const auto matches = pairwise_matches(
          kps1.cast<float>(), kps2.cast<float>(), dsc1.cast<float>(),
          dsc2.cast<float>(), query_rgb_img, ref_rgb_img);

      // Unproject visual matches to obtain 3D point matches
      auto query_kdtree = nanoflann_wrapper(new_pcd.get_cloud());
      auto [current_target_points, current_source_points] = unproject_matches(
          kps1, kps2, matches, sensors[0].rgbd_img, ref_sensor.rgbd_img,
          pose * sensors[0].camera_extrinsics,
          ref_dataset.get_pose(ref_frame_idx) * ref_sensor.camera_extrinsics,
          sensors[0].camera_intrinsics, ref_sensor.camera_intrinsics,
          is_point_associated, ref_kdtree, query_kdtree, 0.5, cfg.depth_min_th,
          cfg.depth_max_th);
      source_points.insert(source_points.end(), current_source_points.begin(),
                           current_source_points.end());
      target_points.insert(target_points.end(), current_target_points.begin(),
                           current_target_points.end());
    }

    // Stop when we reach the number of frames to deal with
    if (current_frame_idx == cfg.n_frames - 1) {
      break;
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

  // Final logging
  std::cout << std::endl << "DONE: " << std::endl;
  std::cout << source_points.size() << " associations found" << std::endl;

  // Save the associations and poses
  PATHoBotDataset::save_kitty_poses(output_folder / output_poses_filename,
                                    current_dataset.get_poses());
  assert(source_points.size() == target_points.size());
  const size_t n_matches = source_points.size();
  std::fstream matches_file(output_folder / output_matches_filename,
                            std::ios_base::out);
  for (int i = 0; i < n_matches; ++i) {
    matches_file << source_points[i].transpose() << " "
                 << target_points[i].transpose() << std::endl;
  }

  // Visualize the matches
  if (visualize)
    ref_map_complete.visualize_with_matches(source_points, target_points);
}
