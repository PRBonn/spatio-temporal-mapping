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
#include <open3d/camera/PinholeCameraParameters.h>
#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/geometry/VoxelGrid.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/visualization/utility/DrawGeometry.h>
#include <open3d/visualization/visualizer/Visualizer.h>

#include <atomic>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "DeformationGraph.hpp"
#include "VoxelizedPointCloud.hpp"
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
      "Odometry using a previous map and deformation of the same in order to "
      "obtain the new plants shhape."};
  std::string dataset_main_folder = "dataset/";
  std::string cfg_filename = "config/config.yaml";
  int ref_number = 1;
  int query_number = 2;
  int row_number = 3;
  bool visualize = false;
  app.add_option(
      "-d,--dataset-folder", dataset_main_folder,
      "The path to the folder where it is contained the dataset to deal with.");
  app.add_option("-r,--ref-number", ref_number,
                 "Number of the dataset to use as reference");
  app.add_option("-q,--query-number", query_number,
                 "Number of the dataset to use as query");
  app.add_option("--row-number", row_number, "Number of the row to use");
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
      ref_dataset_folder / ("mapping_out/voxelized_map.ply");
  fs::path ref_poses_filename =
      ref_dataset_folder / "mapping_out/mapping_poses.txt";
  fs::path query_map_filename =
      query_dataset_folder / ("mapping_out/voxelized_map_onref_" +
                              std::to_string(ref_number) + ".ply");
  fs::path output_folder = query_dataset_folder / "mapping_out";
  if (!fs::exists(output_folder)) {
    fs::create_directory(output_folder);
  }
  fs::path output_map_filename =
      "deformed_map_superpoint_fromref_" + std::to_string(ref_number) + ".ply";
  fs::path matches_filename =
      output_folder / ("matches_onref_" + std::to_string(ref_number) + ".txt");

  // Load the ground truth
  auto gt_map =
      VoxelizedPointCloud::load(query_map_filename, cfg.map_voxel_size);

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

  // Initialize stuff used for deformation
  std::atomic<bool> deformation_done = false;
  std::thread deformation_thread;
  VoxelizedPointCloud ref_map_deformed =
      VoxelizedPointCloud(cfg.map_voxel_size);
  ref_map_deformed.IntegratePointCloud(ref_map_complete.get_pcd());
  VoxelizedPointCloud matching_pcd =
      VoxelizedPointCloud(cfg.matches_cloud_resolution);
  Eigen::MatrixXd matching_cloud;
  auto ref_deformation_graph =
      DeformationGraph(ref_map_complete, cfg.deformation_graph_resolution,
                       cfg.deformation_graph_connectivity);

  // New stuff for deformation
  std::vector<Eigen::Vector3d> source_points, target_points;

  // Load the matches
  std::fstream matches_file(matches_filename, std::ios_base::in);
  source_points = {};
  target_points = {};

  const auto source_cloud_pts = gt_map.get_points();
  const auto target_cloud_pts = ref_map_deformed.get_points();

  int source_id, target_id;
  std::string line;
  while (std::getline(matches_file, line)) {
    std::istringstream ss(line);
    float x, y, z;
    ss >> x;
    ss >> y;
    ss >> z;
    Eigen::Vector3d new_source_pt(x, y, z);
    ss >> x;
    ss >> y;
    ss >> z;
    Eigen::Vector3d new_target_pt(x, y, z);
    source_points.emplace_back(new_source_pt);
    target_points.emplace_back(new_target_pt);
  }

  if (visualize)
    ref_map_deformed.visualize_with_matches(source_points, target_points,
                                            gt_map.get_pcd(), 1.3, 10);

  // Deformation
  auto [min_x, max_x] = ref_map_deformed.get_limits_positions(0);
  ref_map_deformed.replace_points(
      ref_deformation_graph.deform(source_points, target_points, min_x, max_x));

  // Save results
  VoxelizedPointCloud::save(output_folder / output_map_filename,
                            ref_map_deformed);

  // Visualize the result
  if (visualize) ref_map_deformed.visualize(gt_map.get_pcd());

  return 0;
}
