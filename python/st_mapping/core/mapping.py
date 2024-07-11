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
from typing import Tuple
import numpy as np
from st_mapping.pybind import st_mapping_pybind


class PointCloud:
    def __init__(self, pcd: st_mapping_pybind._PointCloud):
        self._internal_pcd = pcd

    def get_points_and_colors(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._internal_pcd._get_points_and_colors()


def point_cloud_from_points_and_colors(
    points: np.ndarray, colors: np.ndarray
) -> PointCloud:
    return PointCloud(
        st_mapping_pybind._PointCloud(
            st_mapping_pybind._Vector3dVector(points),
            st_mapping_pybind._Vector3dVector(colors),
        )
    )


def extract_point_cloud(
    rgb_img: np.ndarray,
    depth_img: np.ndarray,
    intrinsics: np.ndarray,
    extrinsics: np.ndarray,
    min_th: float,
    max_th: float,
    stride: int,
) -> PointCloud:
    return PointCloud(
        st_mapping_pybind._extract_point_cloud(
            rgb_img, depth_img, intrinsics, extrinsics, min_th, max_th, stride
        )
    )


def voxel_down_sample(pcd: PointCloud, voxel_size: float) -> PointCloud:
    return PointCloud(
        st_mapping_pybind._voxel_down_sample(pcd._internal_pcd, voxel_size)
    )


def threshold(pcd: PointCloud, axis: int, min_th: float, max_th: float):
    return PointCloud(
        st_mapping_pybind._threshold(pcd._internal_pcd, axis, min_th, max_th)
    )


def unproject_2d_points(
    points_2d: np.ndarray,
    depth_img: np.ndarray,
    intrinsics: np.ndarray,
    extrinsics: np.ndarray,
    pose: np.ndarray,
    min_th: float,
    max_th: float,
) -> np.ndarray:

    return st_mapping_pybind._unproject_2d_points(
        st_mapping_pybind._Vector2iVector(points_2d),
        depth_img,
        intrinsics,
        extrinsics,
        pose,
        min_th,
        max_th,
    )


def filter_matches(
    target_points_2d: np.ndarray, ref_points_2d: np.ndarray, distance_th: float
) -> Tuple[np.ndarray, np.ndarray]:
    return st_mapping_pybind._filter_matches(
        st_mapping_pybind._Vector3dVector(target_points_2d),
        st_mapping_pybind._Vector3dVector(ref_points_2d),
        distance_th,
    )
