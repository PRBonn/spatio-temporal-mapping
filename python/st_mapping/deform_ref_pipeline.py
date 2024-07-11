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

import numpy as np
from st_mapping.config.config import StMappingConfig
from st_mapping.tools import (
    visualize_3dmatches,
    visualize_aligned_point_clouds,
    save_kitti_poses,
    save_point_cloud,
)
from st_mapping.odometry import OdometryOnRef

from st_mapping.core.metrics import (
    sequence_error,
    absolute_trajectory_error,
    compute_registration_metrics,
)
from st_mapping.core.mapping import (
    point_cloud_from_points_and_colors,
    extract_point_cloud,
    filter_matches,
    unproject_2d_points,
)
from st_mapping.superpoint.image_matcher import ImageMatcher
from st_mapping.core.deformation import DeformationGraph
from tqdm.auto import trange
import time


class DeformRefPipeline:
    def __init__(self, dataset, ref_dataset, config: StMappingConfig):
        self._dataset = dataset
        self._ref_dataset = ref_dataset
        self._config = config

        self._poses = []
        self._exec_times = []
        self._deformation_exec_time = 0.0

        self._n_frames = len(dataset)
        if self._config.n_frames > 1 and self._config.n_frames < self._n_frames:
            self._n_frames = self._config.n_frames

        self._ref_pcd = self._ref_dataset.load_map()
        self._deformed_ref_pcd = None
        self._odometry = OdometryOnRef(
            config,
            self._ref_pcd,
            self._ref_dataset.get_pose(0),
            self._dataset.get_extrinsics(),
        )

        self._image_matcher = ImageMatcher(config)
        self._matched_points = np.empty((0, 3), dtype=np.float64)
        self._ref_matched_points = np.empty((0, 3), dtype=np.float64)
        self._matching_frequency = config.image_matcher.matching_frequency

    def run(self):
        self._run_pipeline()
        self._visualize_matches()
        self._deform()
        self._save_results()
        self._visualize_map()
        self._run_evaluation()
        return

    def _run_pipeline(self):
        for idx in trange(0, self._n_frames, unit="frames", dynamic_ncols=True):
            rgb_img, depth_img = self._dataset[idx]
            start_time = time.perf_counter_ns()
            frame = extract_point_cloud(
                rgb_img,
                depth_img,
                self._dataset.get_intrinsic(),
                self._dataset.get_extrinsics(),
                self._config.dataset.depth_min_th,
                self._config.dataset.depth_max_th,
                self._config.dataset.image_stride,
            )
            _, pose = self._odometry.register_frame(frame)
            # TODO: handle this in another class?
            if idx % self._matching_frequency == 0:
                self._compute_deformation_matches(rgb_img, depth_img, pose)
            self._poses.append(pose)
            self._exec_times.append(time.perf_counter_ns() - start_time)

    def _compute_deformation_matches(
        self, rgb_img: np.ndarray, depth_img: np.ndarray, pose: np.ndarray
    ):
        # Match
        ref_rgb_img, ref_depth_img, ref_pose = (
            self._ref_dataset.get_nearest_image_and_pose(pose)
        )
        points_2d, ref_points_2d = self._image_matcher.match(
            rgb_img, ref_rgb_img, visualize=False
        )

        # Unproject matches
        points_3d = unproject_2d_points(
            points_2d,
            depth_img,
            self._dataset.get_intrinsic(),
            self._dataset.get_extrinsics(),
            pose,
            self._config.dataset.depth_min_th,
            self._config.dataset.depth_max_th,
        )
        ref_points_3d = unproject_2d_points(
            ref_points_2d,
            ref_depth_img,
            self._ref_dataset.get_intrinsic(),
            self._ref_dataset.get_extrinsics(),
            ref_pose,
            self._config.dataset.depth_min_th,
            self._config.dataset.depth_max_th,
        )

        # Filter matches
        points_3d, ref_points_3d = filter_matches(
            points_3d,
            ref_points_3d,
            self._config.image_matcher.matching_distance_th,
        )

        # Save matches
        self._matched_points = np.append(self._matched_points, points_3d, axis=0)
        self._ref_matched_points = np.append(
            self._ref_matched_points, ref_points_3d, axis=0
        )

    def _visualize_matches(self):
        points, colors = self._ref_pcd.get_points_and_colors()
        visualize_3dmatches(
            points, colors, self._matched_points, self._ref_matched_points
        )

    def _deform(self):
        points, colors = self._ref_pcd.get_points_and_colors()
        deformation_graph = DeformationGraph(
            points,
            self._config.deformation.deformation_graph_resolution,
            self._config.deformation.deformation_graph_connectivity,
            self._config.deformation.graph_consistency_weight,
            self._config.deformation.deformation_weight,
        )
        deformation_graph.add_measurements(
            self._ref_matched_points, self._matched_points
        )
        print("DEFORMING...")
        deformation_starting_time = time.perf_counter_ns()
        deformed_points = deformation_graph.deform(
            self._config.deformation.max_num_iterations
        )
        self._deformation_exec_time = time.perf_counter_ns() - deformation_starting_time
        print("...DEFORMED")
        self._deformed_ref_pcd = point_cloud_from_points_and_colors(
            deformed_points, colors
        )

    def _save_results(self):
        poses_path = self._dataset.get_folder_path() / "poses.txt"
        save_kitti_poses(poses_path, np.array(self._poses))
        print("Poses saved at:", poses_path)
        pcd_path = self._dataset.get_folder_path() / "deformed_map.ply"
        if self._deformed_ref_pcd is not None:
            points, colors = self._deformed_ref_pcd.get_points_and_colors()
            save_point_cloud(pcd_path, points, colors)
            print("Map saved at", pcd_path)

    def _visualize_map(self):
        points, colors = self._ref_pcd.get_points_and_colors()
        deformed_points, deformed_colors = (
            self._deformed_ref_pcd.get_points_and_colors()
        )
        visualize_aligned_point_clouds(deformed_points, deformed_colors, points, colors)

    def _run_evaluation(self):
        # Run estimation metrics evaluation, only when GT data was provided
        if self._dataset._has_gt:
            avg_tra, avg_rot = sequence_error(self._dataset.get_gt_poses(), self._poses)
            ate_rot, ate_tra = absolute_trajectory_error(
                self._dataset.get_gt_poses(), self._poses
            )
            print(f"\tAverage Translation Error:\t {avg_tra:6.3f}\t %")
            print(f"\tAverage Rotational Error:\t {avg_rot:6.3f}\t deg/m")
            print(f"\tAbsolute Trajectory Error:\t {ate_tra:6.3f}\t m")
            print(f"\tAbsolute Rotational Error:\t {ate_rot:6.3f}\t rad")

        # Run deformation evaluation, if we don't have errors in the deformation
        # and GT data is provided
        if self._deformed_ref_pcd is not None and self._dataset.has_map():
            gt_points, _ = self._dataset.load_map().get_points_and_colors()
            points, _ = self._ref_pcd.get_points_and_colors()
            deformed_points, _ = self._deformed_ref_pcd.get_points_and_colors()
            fitness_before, rmse_before = compute_registration_metrics(
                points, gt_points, self._config.mapping.voxel_size
            )
            fitness_after, rmse_after = compute_registration_metrics(
                deformed_points, gt_points, self._config.mapping.voxel_size
            )
            print(f"\tFitness before deformation:\t {fitness_before:6.3f}\t")
            print(f"\tRMSE before deformation:\t {rmse_before:6.4f}\t")
            print(f"\tFitness after deformation:\t {fitness_after:6.3f}\t")
            print(f"\tRMSE after deformation:\t\t {rmse_after:6.4f}\t")

        # Run timing metrics evaluation, always
        def _get_fps():
            total_time_s = sum(self._exec_times) * 1e-9
            return float(len(self._exec_times) / total_time_s)

        avg_fps = int(np.ceil(_get_fps()))
        avg_ms = int(np.ceil(1e3 * (1 / _get_fps())))
        def_s = self._deformation_exec_time * 1e-9
        print(f"\tDeformation Runtime:\t\t {def_s:6.3f} s")
        print(f"\tAverage Frequency:\t\t {avg_fps} Hz")
        print(f"\tAverage Runtime:\t\t {avg_ms} ms")
