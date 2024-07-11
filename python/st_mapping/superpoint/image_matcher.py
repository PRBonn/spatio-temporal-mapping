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
import cv2
import torch
import numpy as np

from st_mapping.config import StMappingConfig
from st_mapping.superpoint.superpoint import SuperPointFrontend, match_keypoints
from st_mapping.tools import visualize_visual_matches


class ImageMatcher:

    def __init__(self, config: StMappingConfig):
        self._imgs_size = (
            config.image_matcher.imgs_height,
            config.image_matcher.imgs_width,
        )
        self._superpoint = SuperPointFrontend(
            config.image_matcher.superpoint_weights_path,
            config.image_matcher.non_maximum_suppression,
            config.image_matcher.confidence_th,
            config.image_matcher.nn_th,
            torch.cuda.is_available(),
        )
        self._homography_confidence_th = config.image_matcher.homography_confidence_th

    def match(
        self, source_img: np.ndarray, ref_img: np.ndarray, visualize: bool = False
    ) -> Tuple[np.ndarray, np.ndarray]:
        source_scale_factors = (
            source_img.shape[0] / float(self._imgs_size[0]),
            source_img.shape[1] / float(self._imgs_size[1]),
        )
        ref_scale_factors = (
            ref_img.shape[0] / float(self._imgs_size[0]),
            ref_img.shape[1] / float(self._imgs_size[1]),
        )

        # TODO: the two images can be handled in batch
        source_pts, source_desc = self._superpoint.run(
            cv2.resize(source_img, (self._imgs_size[1], self._imgs_size[0])),
            source_scale_factors,
        )
        ref_pts, ref_desc = self._superpoint.run(
            cv2.resize(ref_img, (self._imgs_size[1], self._imgs_size[0])),
            ref_scale_factors,
        )

        # Match the descriptors
        _, source_pts, ref_pts = match_keypoints(
            source_desc, ref_desc, source_pts, ref_pts, 1
        )

        # Filter out outliers
        _, zorro = cv2.findHomography(
            source_pts.T, ref_pts.T, cv2.USAC_ACCURATE, self._homography_confidence_th
        )
        zorro = zorro.ravel().astype("bool", copy=False)
        pts1 = source_pts[:, zorro].T
        pts2 = ref_pts[:, zorro].T

        if visualize:
            visualize_visual_matches(source_img, ref_img, pts1, pts2)

        return pts1, pts2
