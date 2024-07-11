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
# SOFTWARE.from st_mapping.config.config import StMappingConfig
import numpy as np
from st_mapping.config import StMappingConfig
from st_mapping.pybind import st_mapping_pybind


class AdaptiveThreshold:
    def __init__(self, config: StMappingConfig):
        self._estimator = st_mapping_pybind._AdaptiveThreshold(
            config.registration.initial_threshold,
            config.registration.min_motion_th,
            config.dataset.depth_max_th,
        )

    def get_threshold(self):
        return self._estimator._compute_threshold()

    def update_model_deviation(self, model_deviation: np.ndarray):
        self._estimator._update_model_deviation(model_deviation)
