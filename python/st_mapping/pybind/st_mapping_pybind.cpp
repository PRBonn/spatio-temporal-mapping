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
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "st_mapping/core/Deformation.hpp"
#include "st_mapping/core/GlobalMap.hpp"
#include "st_mapping/core/LocalMap.hpp"
#include "st_mapping/core/Mapping.hpp"
#include "st_mapping/core/Registration.hpp"
#include "st_mapping/core/Threshold.hpp"
#include "st_mapping/metrics/Metrics.hpp"
#include "stl_vector_eigen.h"

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector2i>);
PYBIND11_MAKE_OPAQUE(std::vector<float>);
PYBIND11_MAKE_OPAQUE(st_mapping::PointCloud);

namespace st_mapping {
PYBIND11_MODULE(st_mapping_pybind, m) {
    // Module docstring
    m.doc() = "STMapping core library.";

    auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
        m, "_Vector3dVector", "std::vector<Eigen::Vector3d>",
        py::py_array_to_vectors_double<Eigen::Vector3d>);

    auto vector2ivector = pybind_eigen_vector_of_vector<Eigen::Vector2i>(
        m, "_Vector2iVector", "std::vector<Eigen::Vector2i>",
        py::py_array_to_vectors_int<Eigen::Vector2i>);

    py::bind_vector<std::vector<float>>(m, "_VectorFloat");

    // TODO: improve get_points_and_colors (maybe parallelize it)
    py::class_<PointCloud>(m, "_PointCloud")
        .def(py::init<>())
        .def(py::init([](const std::vector<Eigen::Vector3d> &points,
                         const std::vector<Eigen::Vector3d> &colors) {
                 PointCloud pcd(points.size());
                 std::transform(points.cbegin(), points.cend(), colors.cbegin(), pcd.begin(),
                                [](const Eigen::Vector3d &pt, const Eigen::Vector3d &color) {
                                    return std::make_pair(pt, color);
                                });
                 return pcd;
             }),
             "points"_a, "colors"_a)
        .def("_get_points_and_colors", [](const PointCloud &self) {
            std::vector<Eigen::Vector3d> points, colors;
            points.reserve(self.size());
            colors.reserve(self.size());
            std::for_each(self.cbegin(), self.cend(), [&](const auto &colored_point) {
                const auto &[pt, color] = colored_point;
                points.emplace_back(pt);
                colors.emplace_back(color);
            });
            return std::make_tuple(points, colors);
        });

    py::class_<LocalMap>(m, "_LocalMap")
        .def(py::init([](const Eigen::Matrix4d &camera_extrinsics, const double voxel_size,
                         const double map_size, const int max_points_per_voxel) {
                 Sophus::SE3d camera_extrinsics_sophus(camera_extrinsics);
                 return LocalMap(camera_extrinsics_sophus, voxel_size, map_size,
                                 max_points_per_voxel);
             }),
             "camera_extrinsics"_a, "voxel_size"_a, "map_size"_a, "max_points_per_voxel"_a)
        .def(
            "_integrate_point_cloud",
            [](LocalMap &self, const PointCloud &pcd, const Eigen::Matrix4d &T,
               const bool pose_only_for_resize) {
                Sophus::SE3d pose(T);
                return self.IntegratePointCloud(pcd, pose, pose_only_for_resize);
            },
            "pcd"_a, "T"_a, "pose_only_for_resize"_a)
        .def("_get_points_and_colors", &LocalMap::GetPointsAndColors);

    py::class_<GlobalMap>(m, "_GlobalMap")
        .def(py::init<>())
        .def(
            "_integrate_point_cloud",
            [](GlobalMap &self, const PointCloud &pcd, const Eigen::Matrix4d &T) {
                Sophus::SE3d pose(T);
                self.IntegratePointCloud(pcd, pose);
            },
            "pcd"_a, "T"_a)
        .def("_get_points_and_colors", &GlobalMap::GetPointsAndColors);

    py::class_<DeformationGraph>(m, "_DeformationGraph")
        .def(py::init<const std::vector<Eigen::Vector3d> &, const double, const int, const double,
                      const double>(),
             "cloud"_a, "resolution"_a, "nodes_connectivity"_a, "graph_consistency_weight"_a,
             "deformation_weight"_a)
        .def("_get_graph", &DeformationGraph::GetGraph)
        .def("_add_measurements", &DeformationGraph::AddMeasurements, "source_points"_a,
             "target_points"_a)
        .def("_deform", &DeformationGraph::Deform, "max_num_iterations"_a);

    py::class_<AdaptiveThreshold>(m, "_AdaptiveThreshold")
        .def(py::init<double, double, double>(), "initial_threshold"_a, "min_motion_threshold"_a,
             "max_range"_a)
        .def(
            "_update_model_deviation",
            [](AdaptiveThreshold &self, const Eigen::Matrix4d &T) {
                Sophus::SE3d model_deviation(T);
                self.UpdateModelDeviation(model_deviation);
            },
            "current_deviation"_a)
        .def("_compute_threshold", &AdaptiveThreshold::ComputeThreshold);

    // Core functions
    m.def(
        "_extract_point_cloud",
        [](py::array_t<uint8_t> &rgb_img_py, py::array_t<double> &depth_img_py,
           const Eigen::Matrix3d &camera_intrinsics, const Eigen::Matrix4d &camera_extrinsics,
           const double min_th, const double max_th, const int stride) {
            const auto &rows = rgb_img_py.shape(0);
            const auto &cols = rgb_img_py.shape(1);
            Sophus::SE3d camera_extrinsics_sophus(camera_extrinsics);
            return ExtractPointCloud(cv::Mat(rows, cols, CV_8UC3, rgb_img_py.mutable_data()),
                                     cv::Mat(rows, cols, CV_64FC1, depth_img_py.mutable_data()),
                                     camera_intrinsics, camera_extrinsics_sophus, min_th, max_th,
                                     stride);
        },
        "rgb_img"_a, "depth_img"_a, "camera_intrinsics"_a, "camera_extrinsics"_a, "min_th"_a,
        "max_th"_a, "stride"_a);
    m.def(
        "_unproject_2d_points",
        [](std::vector<Eigen::Vector2i> &points2D, py::array_t<double> &depth_img_py,
           const Eigen::Matrix3d &camera_intrinsics, const Eigen::Matrix4d &camera_extrinsics,
           const Eigen::Matrix4d &pose, const double min_th, const double max_th) {
            const auto &rows = depth_img_py.shape(0);
            const auto &cols = depth_img_py.shape(1);
            const Sophus::SE3d camera_extrinsics_sophus = Sophus::SE3d(camera_extrinsics);
            const Sophus::SE3d pose_sophus = Sophus::SE3d(pose);
            return Unproject2DPoints(
                points2D, cv::Mat(rows, cols, CV_64FC1, depth_img_py.mutable_data()),
                camera_intrinsics, camera_extrinsics_sophus, pose_sophus, min_th, max_th);
        },
        "points2D"_a, "depth_img"_a, "camera_intrinsics"_a, "camera_extrinsics"_a, "pose"_a,
        "min_th"_a, "max_th"_a);
    m.def("_filter_matches", &FilterMatches, "target_points"_a, "ref_points"_a, "distance_th"_a);

    m.def("_voxel_down_sample", &VoxelDownSample, "pcd"_a, "voxel_size"_a);
    m.def("_threshold", &Threshold, "pcd"_a, "axis"_a, "min_th"_a, "max_th"_a);
    m.def(
        "_correct_pose_with_icp",
        [](const PointCloud &frame, const LocalMap &local_map, const Eigen::Matrix4d &T_guess,
           const double max_correspondence_distance, const double kernel_scale,
           int max_num_iterations, double convergence_criterion, int max_num_threads) {
            Sophus::SE3d initial_guess(T_guess);
            return CorrectPoseWithICP(frame, local_map, initial_guess, max_correspondence_distance,
                                      kernel_scale, max_num_iterations, convergence_criterion,
                                      max_num_threads)
                .matrix();
        },
        "frame"_a, "map_pcd"_a, "initial_guess"_a, "max_correspondence_distance"_a,
        "kernel_scale"_a, "max_num_iterations"_a, "convergence_criterion"_a, "max_num_threads"_a);

    // Metrics functions
    m.def("_sequence_error", &metrics::SequenceError, "poses_gt"_a, "poses_result"_a);
    m.def("_absolute_trajectory_error", &metrics::AbsoluteTrajectoryError, "poses_gt"_a,
          "poses_result"_a);
}
}  // namespace st_mapping
