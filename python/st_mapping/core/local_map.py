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
from typing import Tuple

from st_mapping.config.config import StMappingConfig
from st_mapping.pybind import st_mapping_pybind
from st_mapping.core.mapping import PointCloud


class LocalMap:

    def __init__(self, config: StMappingConfig, camera_extrinsics: np.ndarray):
        self._internal_local_map = st_mapping_pybind._LocalMap(
            camera_extrinsics,
            config.mapping.voxel_size,
            config.mapping.local_map_size,
            config.mapping.max_points_per_voxel,
        )

    def integrate_point_cloud(
        self, pcd: PointCloud, pose: np.ndarray, pose_only_for_resize: bool = False
    ):
        return PointCloud(
            self._internal_local_map._integrate_point_cloud(
                pcd._internal_pcd, pose, pose_only_for_resize
            )
        )

    def get_points_and_colors(self) -> Tuple[np.ndarray, np.ndarray]:
        points, colors = self._internal_local_map._get_points_and_colors()
        return np.asarray(points), np.asarray(colors)
