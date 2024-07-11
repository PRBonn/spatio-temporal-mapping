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
from pydantic import BaseModel
from typing import Optional


class DatasetConfig(BaseModel):
    depth_scale: int = 1000
    image_stride: int = 2
    depth_min_th: float = 0.4
    depth_max_th: float = 10.0


class MappingConfig(BaseModel):
    voxel_size: Optional[float] = None
    max_points_per_voxel: int = 10
    local_map_size: Optional[float] = None


class AlignmentConfig(BaseModel):
    stable_features_min_th: float = 0.0
    stable_features_max_th: float = 1.2


class RegistrationConfig(BaseModel):
    initial_threshold: float = 2.0
    min_motion_th: float = 0.01
    max_num_iterations: int = 500
    convergence_criterion: float = 0.0001
    max_num_threads: int = 0  # 0 is max


class ImageMatcherConfig(BaseModel):
    matching_frequency: int = 5
    matching_distance_th: float = 0.5
    imgs_height: int = 220
    imgs_width: int = 260
    superpoint_weights_path: str = "weights/superpoint.pth"
    non_maximum_suppression: int = 4
    confidence_th: float = 0.015
    nn_th: float = 0.7
    homography_confidence_th: int = 30


class DeformationConfig(BaseModel):
    deformation_graph_resolution: float = 0.025
    deformation_graph_connectivity: int = 6
    graph_consistency_weight: float = 100.0
    deformation_weight: float = 10.0
    max_num_iterations: int = 100


class StMappingConfig(BaseModel):
    n_frames: int = -1  # -1 means all
    dataset: DatasetConfig = DatasetConfig()
    mapping: MappingConfig = MappingConfig()
    alignment: AlignmentConfig = AlignmentConfig()
    registration: RegistrationConfig = RegistrationConfig()
    image_matcher: ImageMatcherConfig = ImageMatcherConfig()
    deformation: DeformationConfig = DeformationConfig()
