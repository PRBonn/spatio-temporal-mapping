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

#include <open3d/camera/PinholeCameraIntrinsic.h>
#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/visualization/visualizer/ViewControl.h>
#include <open3d/visualization/visualizer/VisualizerWithKeyCallback.h>

#include <eigen3/Eigen/Dense>

namespace aid4crop {
namespace utils {

using PointCloud = open3d::geometry::PointCloud;
using PointCloud_ptr_const = std::shared_ptr<const PointCloud>;
using PointCloud_ptr = std::shared_ptr<PointCloud>;
using LineSet_ptr = std::shared_ptr<open3d::geometry::LineSet>;
using VisualizerCallbacks = open3d::visualization::VisualizerWithKeyCallback;
using Visualizer = open3d::visualization::Visualizer;
using CameraIntrinsic = open3d::camera::PinholeCameraIntrinsic;
using CameraParams = open3d::camera::PinholeCameraParameters;
using ViewControl = open3d::visualization::ViewControl;

class GlasshouseVisualizer {
 public:
  GlasshouseVisualizer(const PointCloud_ptr_const reference_pcd,
                       const PointCloud_ptr_const keypoints,
                       const Eigen::Matrix3d& camera_intrinsic,
                       const Eigen::Matrix4d& camera_extrinsic,
                       const size_t& height, const size_t& width,
                       const std::string& viewpoint_filename,
                       bool visible = true);

  ~GlasshouseVisualizer();

  // Functions to register new entities
  void registerGroundTruth(const PointCloud_ptr_const ground_truth);

  // Function for visualization
  void updateVisualizer();
  void updateVisualizer(const Eigen::Matrix4d& new_camera_pose);
  void KeepRunning();

 private:
  void _loadViewpoint(const std::string& viewpoint_filename);

  // Callbacks
  void _registerCallbacks();
  void _updateVisualizedGroundTruth();

  VisualizerCallbacks _vis;
  PointCloud_ptr_const _reference_pcd;
  PointCloud_ptr_const _keypoints;
  PointCloud_ptr_const _ground_truth;
  PointCloud_ptr _visualized_ground_truth;
  LineSet_ptr _camera;
  Eigen::Matrix4d _previous_camera_pose;
  bool _groundTruthVisible;
  bool _visible;
};

}  // namespace utils
}  // namespace aid4crop
