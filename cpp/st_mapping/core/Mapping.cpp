// MIT License
//
// Copyright (c) 2024 Luca Lobefaro, Meher V.R. Malladi, Tiziano Guadagnino, Cyrill Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "Mapping.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/parallel_for.h>
#include <tsl/robin_set.h>

#include <Eigen/Dense>
#include <execution>
#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

using namespace oneapi::tbb;

namespace {
using namespace st_mapping;

using Voxel = Eigen::Vector3i;
struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};

Voxel PointToVoxel(const Eigen::Vector3d &point, const double voxel_size) {
    return Voxel(static_cast<int>(std::floor(point.x() / voxel_size)),
                 static_cast<int>(std::floor(point.y() / voxel_size)),
                 static_cast<int>(std::floor(point.z() / voxel_size)));
}

std::vector<int> get_valid_depth_values(const cv::Mat &depth_img,
                                        const float min_th,
                                        const float max_th,
                                        const int stride) {
    std::vector<int> valid_pixels;
    valid_pixels.reserve(depth_img.cols * depth_img.rows);
    for (int i = 0; i < depth_img.rows; i += stride) {
        for (int j = 0; j < depth_img.cols; j += stride) {
            const double &depth_val = depth_img.at<double>(i, j);
            if (depth_val > 0.0 && depth_val > min_th && depth_val < max_th) {
                valid_pixels.emplace_back(i * depth_img.cols + j);
            }
        }
    }
    valid_pixels.shrink_to_fit();
    return valid_pixels;
}

}  // namespace

namespace st_mapping {

PointCloud ExtractPointCloud(const cv::Mat &rgb_img,
                             const cv::Mat &depth_img,
                             const Eigen::Matrix3d &camera_intrinsics,
                             const Sophus::SE3d &camera_extrinsics,
                             const double min_th,
                             const double max_th,
                             const int stride) {
    // Initialization
    PointCloud pcd = PointCloud();
    const auto &K_inv = camera_intrinsics.inverse();

    // Count how many pixels contain invalid depth values and inside the
    // threhsolds and linearize the u, v coordinates to point vector ids
    const auto valid_depth_pixels = get_valid_depth_values(depth_img, min_th, max_th, stride);
    const size_t num_valid_pixels = valid_depth_pixels.size();
    pcd.resize(num_valid_pixels);

    // Extract point cloud
    parallel_for(blocked_range<int>(0, num_valid_pixels), [&](const blocked_range<int> &block) {
        for (int i = block.begin(); i < block.end(); i++) {  // Access row-wise
            // Get the uv coordinates
            const int &linear_index_image = valid_depth_pixels[i];
            const int u = linear_index_image % rgb_img.cols;
            const int v = linear_index_image / rgb_img.cols;

            // Get references to the current point
            auto &[pt, color] = pcd[i];

            // Compute the point position
            const double &depth = depth_img.at<double>(v, u);
            const Eigen::Vector3d uv(u, v, 1);
            pt = camera_extrinsics * (depth * K_inv * uv);

            // Compute the point color
            const auto &bgr_color = rgb_img.at<cv::Vec3b>(v, u);
            color = {bgr_color[2] / 255.0, bgr_color[1] / 255.0, bgr_color[0] / 255.0};
        }
    });

    return pcd;
}

PointCloud VoxelDownSample(const PointCloud &pcd, const double voxel_size) {
    tsl::robin_set<Voxel, VoxelHash> voxels;
    PointCloud pcd_dowsampled;
    pcd_dowsampled.reserve(pcd.size());
    std::for_each(pcd.cbegin(), pcd.cend(), [&](const PointWithColor colored_point) {
        const auto &[point, color] = colored_point;
        const auto voxel = PointToVoxel(point, voxel_size);
        if (!voxels.contains(voxel)) {
            voxels.insert(voxel);
            pcd_dowsampled.emplace_back(colored_point);
        }
    });
    return pcd_dowsampled;
}

PointCloud Threshold(const PointCloud &pcd,
                     const int axis,
                     const double min_th,
                     const double max_th) {
    PointCloud new_pcd;
    new_pcd.reserve(pcd.size());
    std::for_each(pcd.cbegin(), pcd.cend(), [&](const auto &colored_point) {
        const auto &[pt, color] = colored_point;
        if (pt[axis] >= min_th && pt[axis] <= max_th) {
            new_pcd.emplace_back(colored_point);
        }
    });
    new_pcd.shrink_to_fit();
    return new_pcd;
}

std::vector<Eigen::Vector3d> Unproject2DPoints(const std::vector<Eigen::Vector2i> &points2D,
                                               const cv::Mat &depth_img,
                                               const Eigen::Matrix3d &camera_intrinsics,
                                               const Sophus::SE3d &camera_extrinsics,
                                               const Sophus::SE3d &pose,
                                               const double min_th,
                                               const double max_th) {
    // Initialization
    std::vector<Eigen::Vector3d> points3D(points2D.size());
    const auto &K_inv = camera_intrinsics.inverse();
    const Sophus::SE3d T = pose * camera_extrinsics;

    std::transform(std::execution::par, points2D.cbegin(), points2D.cend(), points3D.begin(),
                   [&](const auto &pt2D) {
                       const double &depth = depth_img.at<double>(pt2D[1], pt2D[0]);
                       if (depth > 0.0 && depth > min_th && depth < max_th) {
                           const Eigen::Vector3d uv(pt2D[0], pt2D[1], 1);
                           return Eigen::Vector3d(T * (depth * K_inv * uv));
                       } else {
                           return Eigen::Vector3d(Eigen::Vector3d::Ones() *
                                                  std::numeric_limits<double>::max());
                       }
                   });

    return points3D;
}

Matches3D FilterMatches(const std::vector<Eigen::Vector3d> &target_points,
                        const std::vector<Eigen::Vector3d> &ref_points,
                        const double &distance_th) {
    // Pre-conditions
    if (target_points.size() != ref_points.size()) {
        std::cout << "[ERROR] When filtering matches, the number of 3D points in target and ref "
                     "should match."
                  << std::endl;
        exit(1);
    }

    // Initialization
    std::vector<Eigen::Vector3d> target_points3D_filtered;
    std::vector<Eigen::Vector3d> ref_points3D_filtered;
    std::vector<double> distances;
    std::vector<size_t> ids(target_points.size());
    std::iota(ids.begin(), ids.end(), 0);

    // Compute matches distances
    distances.reserve(target_points.size());
    std::for_each(ids.cbegin(), ids.cend(), [&](const size_t &idx) {
        const Eigen::Vector3d &pt1 = target_points.at(idx);
        const Eigen::Vector3d &pt2 = ref_points.at(idx);
        if (std::isinf(pt1.norm()) || std::isinf(pt2.norm())) {
            return;
        }
        const double distance = (pt1 - pt2).norm();
        if (distance > distance_th) {
            return;
        }
        distances.emplace_back(distance);
    });
    distances.shrink_to_fit();

    // Compute mean and standard deviation of the distances
    const double mean = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    std::vector<double> diff(distances.size());
    std::transform(distances.begin(), distances.end(), diff.begin(),
                   [mean](double x) { return x - mean; });
    const double std_deviation = std::sqrt(
        std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0) / distances.size());
    const double threshold_max = mean + 1 * std_deviation;

    // Filter points based on distances
    target_points3D_filtered.reserve(target_points.size());
    ref_points3D_filtered.reserve(ref_points.size());
    std::for_each(ids.cbegin(), ids.cend(), [&](const size_t &idx) {
        const Eigen::Vector3d &pt1 = target_points.at(idx);
        const Eigen::Vector3d &pt2 = ref_points.at(idx);
        if (std::isinf(pt1.norm()) || std::isinf(pt2.norm())) {
            return;
        }
        const double distance = (pt1 - pt2).norm();
        if (distance > threshold_max) {
            return;
        }
        target_points3D_filtered.emplace_back(pt1);
        ref_points3D_filtered.emplace_back(pt2);
    });
    target_points3D_filtered.shrink_to_fit();
    ref_points3D_filtered.shrink_to_fit();

    return std::make_pair(target_points3D_filtered, ref_points3D_filtered);
}

}  // namespace st_mapping
