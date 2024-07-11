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
from typing import Tuple, Dict
import numpy as np

from st_mapping.pybind import st_mapping_pybind


class DeformationGraph:
    def __init__(
        self,
        points: np.ndarray,
        resolution: float,
        nodes_connectivity: int,
        graph_consistency_weight: float,
        deformation_weight: float,
    ):
        self._internal_deformation_graph = st_mapping_pybind._DeformationGraph(
            st_mapping_pybind._Vector3dVector(points),
            resolution,
            nodes_connectivity,
            graph_consistency_weight,
            deformation_weight,
        )

    def get_graph(self) -> Tuple[np.ndarray, Dict]:
        return self._internal_deformation_graph._get_graph()

    def add_measurements(self, points1: np.ndarray, points2: np.ndarray):
        self._internal_deformation_graph._add_measurements(
            st_mapping_pybind._Vector3dVector(points1),
            st_mapping_pybind._Vector3dVector(points2),
        )

    def deform(self, max_num_iterations: int) -> np.ndarray:
        return self._internal_deformation_graph._deform(max_num_iterations)
