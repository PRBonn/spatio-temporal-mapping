# ----------------------------------------------------------------------------
# NOTE: This file has been adapted from the Kiss-ICP project, but copyright
# still belongs to Kiss-ICP. All rights reserved
# ----------------------------------------------------------------------------
# -              Kiss-ICP: https://github.com/PRBonn/kiss-icp                -
# ----------------------------------------------------------------------------
# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from typing import List, Tuple

import numpy as np
import open3d as o3d

from st_mapping.pybind import st_mapping_pybind


def sequence_error(
    gt_poses: np.ndarray, results_poses: List[np.ndarray]
) -> Tuple[float, float]:
    """Sptis the sequence error for a given trajectory in camera coordinate frames."""
    return st_mapping_pybind._sequence_error(gt_poses, results_poses)


def absolute_trajectory_error(
    gt_poses: np.ndarray, results_poses: List[np.ndarray]
) -> Tuple[float, float]:
    """Sptis the sequence error for a given trajectory in camera coordinate frames."""
    return st_mapping_pybind._absolute_trajectory_error(gt_poses, results_poses)


def compute_registration_metrics(
    points: np.ndarray, gt_points: np.ndarray, voxel_size: float
) -> Tuple[float, float]:
    threshold = voxel_size
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    gt_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(gt_points))
    results = o3d.pipelines.registration.evaluate_registration(pcd, gt_pcd, threshold)
    return results.fitness, results.inlier_rmse
