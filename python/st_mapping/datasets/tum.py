# ----------------------------------------------------------------------------
# NOTE: This file has been adapted from the Kiss-ICP project, but copyright
# still belongs to Kiss-ICP. All rights reserved
# ----------------------------------------------------------------------------
# -              Kiss-ICP: https://github.com/PRBonn/kiss-icp                -
# ----------------------------------------------------------------------------
# MIT License
#
# # Copyright (c) 2023 Saurabh Gupta, Ignacio Vizzo, Cyrill Stachniss, University of Bonn
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
import os
from pathlib import Path
import sys
import numpy as np
from pyquaternion import Quaternion
import cv2

from st_mapping.tools import load_kitti_poses, load_camera_parameters


class TumDataset:
    """TumDataset.
    This dataset expects the following folder structure:
        main_folder:
            > rgb
                0001.jpg
                0002.jpg
                ...
            > depth
                0001.jpg
                0002.jpg
                ...
            depth.txt
            rgb.txt
            groundtruth.txt
            params.yaml
            poses.txt
    If has_poses = False then we do not expect the file poses.txt to be present.
    Note: the file groundtruth.txt is optional, if it is there, then we can evaluate the system
    in visual odometry mode.
    """

    def __init__(
        self, main_folder: Path, has_poses: bool, depth_scale: float, *_, **__
    ):
        self._main_folder = main_folder
        self._has_poses = has_poses
        self._depth_scale = depth_scale

        self._has_gt = self._check_dataset_consistency()

        self._rgb_frames, self._depth_frames, self._matches = self._load_img_filenames()
        self._intrinsics, self._extrinsics, self._img_width, self._img_heigth = (
            load_camera_parameters(main_folder / "params.yaml")
        )
        self._poses, self._gt_poses = self._load_poses()

    def _check_dataset_consistency(self):
        subfiles = os.listdir(str(self._main_folder))
        if (
            not "rgb" in subfiles
            or not "depth" in subfiles
            or not os.path.isdir(str(self._main_folder / "rgb"))
            or not os.path.isdir(str(self._main_folder / "depth"))
            or not "depth.txt" in subfiles
            or not "rgb.txt" in subfiles
            or not "params.yaml" in subfiles
        ):
            print(
                "[ERROR] Check that the given dataset path contains rgb/ depth/ folders and params.yaml, rgb.txt and depth.txt files"
            )
            sys.exit(1)
        if self._has_poses and not "poses.txt" in subfiles:
            print(
                "[ERROR] No poses in the given dataset. This means, no mapping without visual odometry possible. Please, compute the poses first."
            )
            sys.exit(1)
        return "groundtruth.txt" in subfiles

    def _load_img_filenames(self):
        rgb_frames = np.loadtxt(fname=self._main_folder / "rgb.txt", dtype=str)
        depth_frames = np.loadtxt(fname=self._main_folder / "depth.txt", dtype=str)
        matches = self._get_matches(depth_frames[:, 0], rgb_frames[:, 0])
        return rgb_frames, depth_frames, matches

    def _load_poses(self):
        if self._has_poses:
            poses = load_kitti_poses(self._main_folder / "poses.txt")
            if len(poses) != len(self):
                print("[ERROR] Number of poses and number of images do not match.")
                sys.exit(1)
        else:
            poses = np.array([])

        if self._has_gt:
            gt_list = np.loadtxt(fname=self._main_folder / "groundtruth.txt", dtype=str)
            gt_indices = self._get_matches(
                gt_list[:, 0].astype(np.float64),
                self._depth_frames[:, 0][self._matches[:, 0]].astype(np.float64),
            )
            xyz = gt_list[gt_indices[:, 0]][:, 1:4]

            rotations = np.array(
                [
                    Quaternion(x=x, y=y, z=z, w=w).rotation_matrix
                    for x, y, z, w in gt_list[gt_indices[:, 0]][:, 4:]
                ]
            )
            num_poses = rotations.shape[0]
            gt_poses = (
                np.eye(4, dtype=np.float64).reshape(1, 4, 4).repeat(num_poses, axis=0)
            )
            gt_poses[:, :3, :3] = rotations
            gt_poses[:, :3, -1] = xyz

        else:
            gt_poses = np.array([])

        return poses, gt_poses

    def _get_matches(self, src_timstamps, target_timstamps):
        indices = np.abs(
            (
                np.subtract.outer(
                    src_timstamps.astype(np.float64),
                    target_timstamps.astype(np.float64),
                )
            )
        )
        src_matches = np.arange(len(src_timstamps))
        target_matches = np.argmin(indices, axis=1)
        _, unique_indxs = np.unique(target_matches, return_index=True)
        matches = np.vstack((src_matches[unique_indxs], target_matches[unique_indxs])).T
        return matches

    def __len__(self):
        return len(self._matches)

    def __getitem__(self, idx):
        return self.get_image(idx)

    def get_folder_path(self) -> Path:
        return self._main_folder

    def get_image(self, idx):
        if idx < 0 or idx > len(self):
            print(
                f"[ERROR] {idx} is an invalid dataset index, dataset len: {len(self)}."
            )
        depth_idx, rgb_idx = self._matches[idx]
        rgb = cv2.imread(
            str(self._main_folder / self._rgb_frames[rgb_idx][-1]), cv2.IMREAD_COLOR
        )
        depth_unprocessed = cv2.imread(
            str(self._main_folder / self._depth_frames[depth_idx][-1]),
            cv2.IMREAD_UNCHANGED,
        )
        depth = (depth_unprocessed / self._depth_scale).astype(np.float32)
        return rgb, depth

    def img_width(self):
        return self._img_width

    def img_height(self):
        return self._img_heigth

    def get_pose(self, idx):
        if idx < 0 or idx > len(self._poses):
            print(
                f"[ERROR] {idx} is an invalid pose index (check that this dataset has poses)."
            )
        return self._poses[idx]

    def get_gt_pose(self, idx):
        if idx < 0 or idx > len(self._gt_poses):
            print(
                f"[ERROR] {idx} is an invalid ground truth pose index (check that this dataset has gt_poses)."
            )
        return self._gt_poses[idx]

    def get_poses(self):
        if not self._has_poses:
            print(f"[ERROR] This dataset does not have poses.")
        return self._poses

    def get_gt_poses(self):
        if not self._has_gt:
            print(f"[ERROR] This dataset does not have ground truth poses.")
        return self._gt_poses

    def get_intrinsic(self):
        return self._intrinsics

    def get_extrinsics(self):
        return self._extrinsics

    def has_gt(self):
        return self._has_gt
