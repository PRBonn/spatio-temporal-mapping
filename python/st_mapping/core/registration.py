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

from st_mapping.pybind import st_mapping_pybind

from st_mapping.core.mapping import PointCloud
from st_mapping.core.local_map import LocalMap


def correct_pose_with_icp(
    frame: PointCloud,
    local_map: LocalMap,
    initial_guess: np.ndarray,
    max_correspondence_distance: float,
    kernel_scale: float,
    max_num_iterations: int,
    convergence_criterion: float,
    max_num_threads: int,
) -> np.ndarray:
    return st_mapping_pybind._correct_pose_with_icp(
        frame._internal_pcd,
        local_map._internal_local_map,
        initial_guess,
        max_correspondence_distance,
        kernel_scale,
        max_num_iterations,
        convergence_criterion,
        max_num_threads,
    )
