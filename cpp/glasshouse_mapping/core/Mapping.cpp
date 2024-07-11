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
// Some part of this code is strongly inspired from Open3D source code, in
// particular the functions "extract_point_cloud" and "get_valid_depth_values"
// https://github.com/isl-org/Open3D
#include <oneapi/tbb.h>
#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/concurrent_hash_map.h>
#include <open3d/geometry/Image.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/RGBDImage.h>

#include <Mapping.hpp>
#include <cstddef>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numeric>
#include <tuple>
#include <unordered_map>

#include "SensorInfo.hpp"
#include "VoxelizedPointCloud.hpp"
#include "utils/nanoflann/nanoflannWrapper.hpp"
#include "utils/voxel_hash_map/VoxelHashMap.hpp"

using namespace oneapi::tbb;

using VectorInt = std::vector<int>;
namespace aid4crop {
namespace core {

VectorInt get_valid_depth_values(const open3d::geometry::Image& depth,
                                 const float min_th, const float max_th,
                                 const int stride) {
  VectorInt valid_pixels;
  int num_pixels = depth.width_ * depth.height_;
  valid_pixels.reserve(num_pixels);
  for (int i = 0; i < depth.height_; i += stride) {
    for (int j = 0; j < depth.width_; j += stride) {
      const float* depth_ptr = depth.PointerAt<float>(j, i);
      if (*depth_ptr > 0 && *depth_ptr > min_th && *depth_ptr < max_th) {
        valid_pixels.emplace_back(i * depth.width_ + j);
      }
    }
  }
  valid_pixels.shrink_to_fit();
  return valid_pixels;
}

PointCloud_ptr extract_point_cloud_single_sensor(
    const open3d::geometry::RGBDImage& rgbd_img,
    const open3d::camera::PinholeCameraIntrinsic& camera_intrinsics,
    const float min_th, const float max_th, const int stride) {
  // Initialization
  auto pcd = std::make_shared<open3d::geometry::PointCloud>();
  const auto K_inv = camera_intrinsics.intrinsic_matrix_.inverse();

  // Count how many pixels contain invalid depth values and inside the
  // threhsolds and linearize the u, v coordinates to point vector ids
  const auto valid_depth_pixels =
      get_valid_depth_values(rgbd_img.depth_, min_th, max_th, stride);
  const size_t num_valid_pixels = valid_depth_pixels.size();
  pcd->points_.resize(num_valid_pixels);
  pcd->colors_.resize(num_valid_pixels);

  // Extract point cloud
  parallel_for(
      blocked_range<int>(0, num_valid_pixels),
      [&](const blocked_range<int>& block) {
        for (int i = block.begin(); i < block.end(); i++) {  // Access row-wise
          // Get the uv coordinates
          const int linear_index_image = valid_depth_pixels[i];
          const int u = linear_index_image % rgbd_img.depth_.width_;
          const int v = linear_index_image / rgbd_img.depth_.width_;

          // Compute the point position
          float depth = *rgbd_img.depth_.PointerAt<float>(u, v);
          Eigen::Vector3d uv(u, v, 1);
          pcd->points_[i] = depth * K_inv * uv;

          // Compute the point color
          const auto rgb_ptr = rgbd_img.color_.PointerAt<uint8_t>(u, v, 0);
          pcd->colors_[i] =
              Eigen::Vector3d(*rgb_ptr, *(rgb_ptr + 1), *(rgb_ptr + 2)) / 255;
        }
      });

  return pcd;
}

VoxelizedPointCloud extract_point_cloud(std::vector<SensorInfo>& sensors,
                                        const Eigen::Matrix4d& sensors_pose,
                                        const float voxel_size,
                                        const float min_th, const float max_th,
                                        const int stride, const int min_width,
                                        const int max_width) {
  // Initialization
  auto voxelized_pcd = VoxelizedPointCloud(voxel_size);

  // Extract the point cloud from each sensor in sensor base frame
  uint sensor_idx = 0;
  for (auto& sensor : sensors) {
    // Extract the point cloud
    auto current_pcd = extract_point_cloud_single_sensor(
        *sensor.rgbd_img, sensor.camera_intrinsics, min_th, max_th, stride);

    // Bring the point cloud in world frame
    current_pcd->Transform(sensors_pose * sensor.camera_extrinsics);

    // Expand the point cloud that will contain all the sensors' points
    voxelized_pcd.IntegratePointCloud(*current_pcd);

    ++sensor_idx;
  }

  // Remove outliers from point cloud
  voxelized_pcd = clean_point_cloud(voxelized_pcd);

  return voxelized_pcd;
}

std::vector<size_t> integrate_mask(const std::vector<size_t>& source,
                                   const std::vector<size_t>& mask) {
  std::vector<size_t> out;
  out.reserve(mask.size());
  for (const auto& el : mask) {
    out.emplace_back(source[el]);
  }
  return out;
}

VoxelizedPointCloud clean_point_cloud(const VoxelizedPointCloud& pcd) {
  // Decomment the two under to be FAST
  auto clean_pcd = pcd.UniformDownSample(6);
  /* return clean_pcd.RemoveRadiusOutliers(20, 0.1); */
  return clean_pcd;

  // Decomment the two under to be DENSE
  /* auto clean_pcd = pcd.UniformDownSample(10); */
  /* return clean_pcd.RemoveRadiusOutliers(20, 0.5); */

  // The following line is just to remember
  /* return clean_pcd.RemoveStatisticalOutliers(20, 0.5); */
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
unproject_matches(
    const Eigen::MatrixX2d& kps1, const Eigen::MatrixX2d& kps2,
    const std::vector<std::tuple<int, int, float>>& matches,
    const std::shared_ptr<open3d::geometry::RGBDImage>& rgbd_img1,
    const std::shared_ptr<open3d::geometry::RGBDImage>& rgbd_img2,
    const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2,
    const open3d::camera::PinholeCameraIntrinsic& camera_intrinsics1,
    const open3d::camera::PinholeCameraIntrinsic& camera_intrinsics2,
    std::vector<bool>& is_point_associated, const nanoflann_wrapper& ref_kdtree,
    const nanoflann_wrapper& query_kdtree, const double& distance_th,
    const float& min_z_th, const float& max_z_th) {
  // Initialization
  const size_t n_matches = matches.size();
  std::vector<Eigen::Vector3d> pts1;
  std::vector<Eigen::Vector3d> pts2;
  std::vector<double> distances;
  std::vector<int> points_ids;
  const int img1_rows = rgbd_img1->depth_.height_;
  const int img1_cols = rgbd_img1->depth_.width_;
  const int img2_rows = rgbd_img2->depth_.height_;
  const int img2_cols = rgbd_img2->depth_.width_;
  const Eigen::Matrix3d& R1 = pose1.block<3, 3>(0, 0);
  const Eigen::Vector3d& t1 = pose1.block<3, 1>(0, 3);
  const Eigen::Matrix3d& R2 = pose2.block<3, 3>(0, 0);
  const Eigen::Vector3d& t2 = pose2.block<3, 1>(0, 3);

  // IMPORTANT: kps1 are in the target, kps2 in the source

  // Prepare some lambda and variable
  auto dimensions_img_1 = Eigen::Array2d(img1_cols, img1_rows);
  auto dimensions_img_2 = Eigen::Array2d(img2_cols, img2_rows);
  const Eigen::Matrix3d K_inv1 = camera_intrinsics1.intrinsic_matrix_.inverse();
  const Eigen::Matrix3d K_inv2 = camera_intrinsics2.intrinsic_matrix_.inverse();

  // To perform a 3x3 matrix to 2 vector mltiplication in homogeneous
  // coordinates
  auto hom_mul = [](const Eigen::Matrix3d& A, const Eigen::Vector2d& p) {
    return A.block<3, 2>(0, 0) * p + A.block<3, 1>(0, 2);
  };

  // To unproject a point from image
  auto unproj =
      [&](const Eigen::Vector2d& p_img,
          const std::shared_ptr<open3d::geometry::RGBDImage>& rgbd_img,
          const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
          const Eigen::Matrix3d& K_inv, const nanoflann_wrapper& kdtree) {
        float depth = *rgbd_img->depth_.PointerAt<float>(p_img(0), p_img(1));
        Eigen::Vector3d point3d = R * depth * hom_mul(K_inv, p_img) + t;
        const auto neighbors =
            kdtree.return_k_closest_points_radius(point3d, 1, 0.005);
        bool is_good =
            depth >= min_z_th && depth <= max_z_th && !neighbors.empty();

        return std::make_tuple(point3d, neighbors, is_good);
      };

  // For each match: unproject the two points
  pts1.reserve(n_matches);
  pts2.reserve(n_matches);
  distances.reserve(n_matches);
  points_ids.reserve(n_matches);
  std::for_each(matches.cbegin(), matches.cend(), [&](const auto& match) {
    auto& [query, ref, kps_dist] = match;

    const auto [pt1, nn1, is_good1] =
        unproj(kps1.row(query).array().transpose() * dimensions_img_1,
               rgbd_img1, R1, t1, K_inv1, query_kdtree);
    if (!is_good1) {
      return;
    }
    const auto [pt2, nn2, is_good2] =
        unproj(kps2.row(ref).array().transpose() * dimensions_img_2, rgbd_img2,
               R2, t2, K_inv2, ref_kdtree);
    if (!is_good2) {
      return;
    }

    // Compute distance between points
    double distance = (pt1 - pt2).norm();

    // Mantain only if they are valid
    if (distance < distance_th) {
      pts1.emplace_back(pt1);
      pts2.emplace_back(pt2);
      points_ids.emplace_back(nn2.at(0));
      distances.emplace_back(distance);
    }
  });
  pts1.shrink_to_fit();
  pts2.shrink_to_fit();
  points_ids.shrink_to_fit();
  distances.shrink_to_fit();

  assert(pts1.size() == pts2.size());
  assert(pts1.size() == distances.size());
  assert(pts1.size() == points_ids.size());

  // Compute mean and standard deviation of the distances
  const double mean = std::accumulate(distances.begin(), distances.end(), 0.0) /
                      distances.size();
  std::vector<double> diff(distances.size());
  std::transform(distances.begin(), distances.end(), diff.begin(),
                 [mean](double x) { return x - mean; });
  const double std_deviation = std::sqrt(
      std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) /
      distances.size());
  const double threshold_max = mean + 1 * std_deviation;

  // Filter matches based on distances
  std::vector<Eigen::Vector3d> filtered_pts1, filtered_pts2;
  filtered_pts1.reserve(pts1.size());
  filtered_pts2.reserve(pts2.size());
  for (size_t i = 0; i < pts1.size(); ++i) {
    if (distances[i] < threshold_max && !is_point_associated[points_ids[i]]) {
      filtered_pts1.emplace_back(pts1[i]);
      filtered_pts2.emplace_back(pts2[i]);
      is_point_associated[points_ids[i]] = true;
    }
  }
  filtered_pts1.shrink_to_fit();
  filtered_pts2.shrink_to_fit();

  assert(filtered_pts1.size() == filtered_pts2.size());

  return {filtered_pts1, filtered_pts2};
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
filter_matches(const std::vector<Eigen::Vector3d>& source_pts,
               const std::vector<Eigen::Vector3d>& target_pts,
               const VoxelizedPointCloud& source_pcd,
               const VoxelizedPointCloud& target_pcd) {
  // Initialization
  assert(source_pts.size() == target_pts.size());
  const size_t n_matches = source_pcd.size();
  std::vector<Eigen::Vector3d> filtered_source_pts, filtered_target_pts;
  filtered_source_pts.reserve(n_matches);
  filtered_target_pts.reserve(n_matches);
  auto source_kdtree = nanoflann_wrapper(source_pcd.get_cloud());
  auto target_kdtree = nanoflann_wrapper(target_pcd.get_cloud());

  const double radius = 0.05 / 2;

  for (int i = 0; i < n_matches; ++i) {
    // Check that we have a nn for the source point
    auto source_nn =
        source_kdtree.return_k_closest_points_radius(source_pts[i], 1, radius);
    if (source_nn.size() != 1) continue;
    /* auto source_pt = source_pcd.get_point(source_nn.at(0)); */
    auto source_pt = source_pts[i];

    // Check that we have a nn for the target point
    auto target_nn =
        target_kdtree.return_k_closest_points_radius(target_pts[i], 1, radius);
    if (target_nn.size() != 1) continue;
    /* auto target_pt = target_pcd.get_point(target_nn.at(0)); */
    auto target_pt = target_pts[i];

    // Save this match as good
    filtered_source_pts.emplace_back(source_pt);
    filtered_target_pts.emplace_back(target_pt);
  }

  filtered_source_pts.shrink_to_fit();
  filtered_target_pts.shrink_to_fit();

  return {filtered_source_pts, filtered_target_pts};
}

Eigen::MatrixXd extract_given_keypoints(
    const Eigen::MatrixX2d& keypoints,
    const std::shared_ptr<open3d::geometry::RGBDImage>& rgbd_img,
    const Eigen::Matrix4d& pose,
    const open3d::camera::PinholeCameraIntrinsic& camera_intrinsics) {
  // Initialization
  const size_t n_points = keypoints.rows();
  Eigen::MatrixXd points_3d(n_points, 3);
  const int img_rows = rgbd_img->depth_.height_;
  const int img_cols = rgbd_img->depth_.width_;
  int u, v;
  double z;
  const Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
  const Eigen::Vector3d t = pose.block<3, 1>(0, 3);

  for (size_t i = 0; i < n_points; ++i) {
    u = keypoints(i, 0) * img_rows;
    v = keypoints(i, 1) * img_cols;
    z = (double)*rgbd_img->depth_.PointerAt<float>(v, u);

    points_3d(i, 0) = (v - camera_intrinsics.GetPrincipalPoint().first) * z /
                      camera_intrinsics.GetFocalLength().first;
    points_3d(i, 1) = (u - camera_intrinsics.GetPrincipalPoint().second) * z /
                      camera_intrinsics.GetFocalLength().second;
    points_3d(i, 2) = z;

    points_3d.row(i) = ((R * points_3d.row(i).transpose()) + t);
  }

  return points_3d;
}

}  // namespace core
}  // namespace aid4crop
