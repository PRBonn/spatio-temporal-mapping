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
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/VoxelGrid.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/visualization/utility/DrawGeometry.h>

#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <string>

#include "VoxelizedPointCloud.hpp"
#include "dataset/PATHoBotDataset.hpp"
#include "cilantro_wrapper/CilantroPointCloud.hpp"
#include "utils/config/Config.hpp"
#include "utils/decorators/ExecutionTimerDecorator.hpp"
#include "utils/parser/CLI11.hpp"
#include "utils/parser/PATHoBotDatasetParser.hpp"

using namespace aid4crop::dataset;
using namespace aid4crop::core;
using namespace aid4crop::utils;
using namespace aid4crop::cilantro_wrapper;
namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
  // Parse the command line
  CLI::App app{"This application is the cilantro non-rigid ICP baseline."};
  std::string dataset_main_folder = "dataset/";
  std::string cfg_filename = "config/config.yaml";
  int ref_number = 1;
  int query_number = 2;
  int row_number = 3;
  bool after_sp = false;
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
  app.add_option("--after-sp", after_sp,
                 "If you want to apply non rigid ICP after sp_deform, so on "
                 "the map produced with the superpoint deformation.");
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
  if (after_sp) {
    ref_map_filename =
        query_dataset_folder / ("mapping_out/deformed_map_superpoint_fromref_" +
                                std::to_string(ref_number) + ".ply");
  }
  fs::path query_map_filename =
      query_dataset_folder / ("mapping_out/voxelized_map_onref_" +
                              std::to_string(ref_number) + ".ply");
  fs::path output_folder = query_dataset_folder / "mapping_out";
  fs::path output_map_filename =
      "deformed_map_cilantro_fromref_" + std::to_string(ref_number) + ".ply";
  if (after_sp) {
    output_map_filename = "deformed_map_myapproach_fromref_" +
                          std::to_string(ref_number) + ".ply";
  }

  // Load the reference point cloud
  auto ref_pcd =
      VoxelizedPointCloud::load(ref_map_filename, cfg.map_voxel_size);

  // Load the query point cloud
  auto query_pcd =
      VoxelizedPointCloud::load(query_map_filename, cfg.map_voxel_size);

  // Compute the cilantro version of both pcds
  CilantroPointCloud ref_pcd_cilantro(ref_pcd.get_points());
  CilantroPointCloud query_pcd_cilantro(query_pcd.get_points());

  // Pre-computations
  ref_pcd_cilantro.estimateNormals();
  ref_pcd_cilantro.computeDeformationField();
  query_pcd_cilantro.estimateNormals();

  // Deform reference map
  std::cout << "Non-rigid ICP..." << std::endl;
  auto ref_deformation = compute_execution_time(
      "CilantroPointCloud::non_rigid_icp()",
      [&ref_pcd_cilantro](const CilantroPointCloud& query_pcd_cilantro) {
        return ref_pcd_cilantro.non_rigid_icp(query_pcd_cilantro);
      },
      query_pcd_cilantro);
  ref_pcd.deform(ref_deformation);

  // Save result
  VoxelizedPointCloud::save(output_folder / output_map_filename, ref_pcd);

  // Visualize after non-rigid ICP
  if (visualize) {
    std::cout << "Point clouds after deformation of the reference (colored one)"
              << std::endl;
    ref_pcd.visualize(query_pcd.get_pcd());
  }

  return 0;
}
