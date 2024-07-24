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
# SOFTWARE.
import importlib
import os
from abc import ABC
from functools import partial
from typing import Callable, List

import numpy as np

YELLOW = np.array([1, 0.706, 0])
RED = np.array([128, 0, 0]) / 255.0
BLACK = np.array([0, 0, 0])
WHITE = np.array([1.0, 1.0, 1.0])
BLUE = np.array([0.4, 0.5, 0.9])
ARB = np.array([0.1, 0.5, 0.6])
SPHERE_SIZE = 0.20


class StubVisualizer(ABC):
    def __init__(self):
        pass

    def update(self, pose):
        pass

    def quit(self):
        pass


class PosesVisualizer(StubVisualizer):
    def __init__(self):
        try:
            self._o3d = importlib.import_module("open3d")
        except ModuleNotFoundError as err:
            print(f'open3d is not installed on your system, run "pip install open3d"')
            exit(1)

        # Initialize GUI controls
        self._block_vis = True
        self._play_crun = False

        # Create data
        self._frames = []

        # Initialize visualizer
        self._vis = self._o3d.visualization.VisualizerWithKeyCallback()
        self._register_key_callbacks()
        self._initialize_visualizer()

    def quit(self):
        self._vis.destroy_window()

    def update(self, pose):
        new_frame = self._o3d.geometry.TriangleMesh.create_coordinate_frame(SPHERE_SIZE)
        new_frame.transform(pose)
        self._frames.append(new_frame)

        self._vis.add_geometry(self._frames[-1], reset_bounding_box=False)
        self._vis.reset_view_point(True)

        while self._block_vis:
            self._vis.poll_events()
            self._vis.update_renderer()
            if self._play_crun:
                break
        self._block_vis = not self._block_vis

    def _initialize_visualizer(self):
        w_name = self.__class__.__name__
        self._vis.create_window(window_name=w_name, width=1920, height=1080)
        self._set_black_background(self._vis)
        self._vis.reset_view_point(True)
        self._vis.get_render_option().point_size = 1
        print(
            f"{w_name} initialized. Press:\n"
            "\t[SPACE] to pause/start\n"
            "\t  [ESC] to exit\n"
            "\t    [N] to step\n"
            "\t    [C] to center the viewpoint\n"
            "\t    [W] to toggle a white background\n"
            "\t    [B] to toggle a black background\n"
        )

    def _register_key_callback(self, keys: List, callback: Callable):
        for key in keys:
            self._vis.register_key_callback(ord(str(key)), partial(callback))

    def _register_key_callbacks(self):
        self._register_key_callback(["Ā", "Q", "\x1b"], self._quit)
        self._register_key_callback([" "], self._start_stop)
        self._register_key_callback(["N"], self._next_frame)
        self._register_key_callback(["C"], self._center_viewpoint)
        self._register_key_callback(["B"], self._set_black_background)
        self._register_key_callback(["W"], self._set_white_background)

    def _set_black_background(self, vis):
        vis.get_render_option().background_color = BLACK

    def _set_white_background(self, vis):
        vis.get_render_option().background_color = WHITE

    def _quit(self, vis):
        print("Destroying Visualizer")
        vis.destroy_window()
        os._exit(0)

    def _next_frame(self, vis):
        self._block_vis = not self._block_vis

    def _start_stop(self, vis):
        self._play_crun = not self._play_crun

    def _center_viewpoint(self, vis):
        vis.reset_view_point(True)
