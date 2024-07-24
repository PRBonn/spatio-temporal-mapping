# MIT License
#
# Copyright (c) 2024 Luca Lobefaro, Meher V.R. Malladi, Tiziano Guadagnino, Cyrill Stachniss
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
from re import I

from typing import Tuple, List
import numpy as np
from st_mapping.config.config import StMappingConfig
from st_mapping.core.local_map import LocalMap
from st_mapping.core.mapping import PointCloud, voxel_down_sample, threshold
from st_mapping.core.registration import correct_pose_with_icp
from st_mapping.core.threshold import AdaptiveThreshold


class StubOdometry:
    def __init__(
        self,
        config: StMappingConfig,
        camera_extrinsics: np.ndarray,
        poses: List[np.ndarray],
    ):
        self._config = config
        self._poses = poses
        self._next_pose_idx = 0
        self._local_map = LocalMap(config, camera_extrinsics)

    def register_frame(self, frame: PointCloud) -> Tuple[PointCloud, np.ndarray]:
        pose = self._poses[self._next_pose_idx]
        self._next_pose_idx += 1
        processed_frame = self._preprocess(frame)
        processed_frame = self._local_map.integrate_point_cloud(processed_frame, pose)
        return (processed_frame, pose)

    def _preprocess(self, frame: PointCloud) -> PointCloud:
        return voxel_down_sample(frame, self._config.mapping.voxel_size * 1.5)


class Odometry:
    def __init__(self, config: StMappingConfig, camera_extrinsics: np.ndarray):
        self._config = config
        self._last_pose = np.eye(4)
        self._last_delta = np.eye(4)
        self._local_map = LocalMap(config, camera_extrinsics)
        self._adaptive_threshold = AdaptiveThreshold(self._config)
        self._camera_extrinsics = camera_extrinsics

    def register_frame(self, frame: PointCloud) -> Tuple[PointCloud, np.ndarray]:
        # Initialization
        processed_frame = self._preprocess(frame)
        sigma = self._adaptive_threshold.get_threshold()
        initial_guess = self._last_pose @ self._last_delta

        # Pose estimation
        pose = correct_pose_with_icp(
            processed_frame,
            self._local_map,
            initial_guess,
            3 * sigma,
            sigma / 3,
            self._config.registration.max_num_iterations,
            self._config.registration.convergence_criterion,
            self._config.registration.max_num_threads,
        )

        # Update
        self._adaptive_threshold.update_model_deviation(
            np.linalg.inv(initial_guess) @ pose
        )
        processed_frame = self._local_map.integrate_point_cloud(processed_frame, pose)
        self._last_delta = np.linalg.inv(self._last_pose) @ pose
        self._last_pose = pose

        return processed_frame, pose

    def _preprocess(self, frame: PointCloud) -> PointCloud:
        return voxel_down_sample(frame, self._config.mapping.voxel_size * 1.5)


class OdometryOnRef:
    """
    IMPORTANT: In this class we assume that the reference map was built with the same
    camera extrinsics. If this is not the case, we have a problem.
    """

    def __init__(
        self,
        config: StMappingConfig,
        ref_pcd: PointCloud,
        first_pose: np.ndarray,
        camera_extrinsics: np.ndarray,
    ):
        self._config = config
        self._last_pose = first_pose
        self._last_delta = np.eye(4)
        self._local_map = LocalMap(config, camera_extrinsics)
        self._adaptive_threshold = AdaptiveThreshold(self._config)
        self._camera_extrinsics = camera_extrinsics
        self._ref_pcd = self._preprocess_ref_map(ref_pcd)
        self._local_map.integrate_point_cloud(
            self._ref_pcd, first_pose, pose_only_for_resize=True
        )

    def register_frame(self, frame: PointCloud) -> Tuple[PointCloud, np.ndarray]:
        # Initialization
        processed_frame = self._preprocess_frame(frame)
        sigma = self._adaptive_threshold.get_threshold()
        initial_guess = self._last_pose @ self._last_delta

        # Pose estimation
        pose = correct_pose_with_icp(
            processed_frame,
            self._local_map,
            initial_guess,
            3 * sigma,
            sigma / 3,
            self._config.registration.max_num_iterations,
            self._config.registration.convergence_criterion,
            self._config.registration.max_num_threads,
        )

        # Update
        self._adaptive_threshold.update_model_deviation(
            np.linalg.inv(initial_guess) @ pose
        )
        # TODO: can we improve this stage?
        processed_frame = self._local_map.integrate_point_cloud(processed_frame, pose)
        self._local_map.integrate_point_cloud(
            self._ref_pcd, pose, pose_only_for_resize=True
        )
        self._last_delta = np.linalg.inv(self._last_pose) @ pose
        self._last_pose = pose

        return processed_frame, pose

    def _preprocess_frame(self, frame: PointCloud) -> PointCloud:
        return voxel_down_sample(frame, self._config.mapping.voxel_size * 1.5)

    def _preprocess_ref_map(self, ref_pcd: PointCloud) -> PointCloud:
        return threshold(
            ref_pcd,
            2,
            self._config.alignment.stable_features_min_th,
            self._config.alignment.stable_features_max_th,
        )
